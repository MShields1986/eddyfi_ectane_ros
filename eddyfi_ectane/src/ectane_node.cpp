#include "ectane_node.h"

EctaneNode::

EctaneNode::EctaneNode(bool* kill_now)
     : nh_(),
       rate_(10),
       kill_now_(kill_now),
       node_name_(ros::this_node::getName()),
       ns_(ros::this_node::getNamespace()),
       geom_param_prefix_(ns_ + "/geometry/"),
       crosstalk_param_prefix_(ns_ + "/crosstalk/"),
       ectane_handler_(
           EctaneNode::paramHandler(ns_ + "/settings/ectane_address", ectane_address_),
           EctaneNode::configurationsFromString(
               EctaneNode::paramHandler(ns_ + "/settings/configuration", configuration_str_)
               ),
           EctaneNode::paramHandler(ns_ + "/settings/acquisition_rate", acquisition_rate_),
           EctaneNode::paramHandler(ns_ + "/settings/gain", gain_),
           EctaneNode::paramHandler(ns_ + "/settings/frequency", frequency_),
           EctaneNode::paramHandler(ns_ + "/settings/voltage", voltage_)
       ),
       ec_data_ptr_(ectane_handler_.ec_data_ptr())
{
    getChannelNamespaces();
    getChannelConfigurations();

    // Populate ROS message
    //ec_data_.header.frame_id = ns_; // Include the proceeding slash...
    ec_data_.header.frame_id = "ectane";

    for (auto& entry : channel_configs_and_readings_) {
        ROS_DEBUG_STREAM(
            "Channel: " << std::get<0>(entry) << 
            ", time_slot: " << std::get<1>(entry) << 
            ", input: " << std::get<2>(entry) << 
            ", x: " << std::get<3>(entry) << 
            ", y: " << std::get<4>(entry) << 
            ", imaginary: " << std::get<5>(entry) << 
            ", real: " << std::get<6>(entry)
            );

        eddyfi_ectane::Channel channel = eddyfi_ectane::Channel();
        channel.name = std::get<0>(entry);
        channel.time_slot = std::get<1>(entry);
        channel.input = std::get<2>(entry);
        channel.x_position = std::get<3>(entry);
        channel.y_position = std::get<4>(entry);
        // TODO: Multiple frequency functionality goes here too
        channel.frequency = frequency_;
        channel.dc_voltage = voltage_;

        /*
        int n_timeslots = end(ec_data_.time_slots) - begin(ec_data_.time_slots);
        if (n_timeslots < std::get<1>(entry)) {
            ec_data_.time_slots.push_back(eddyfi_ectane::TimeSlot());
        }

        ec_data_.time_slots[std::get<1>(entry) - 1].channels.push_back(channel);
        */
        ec_data_.channels.push_back(channel);

    }

    ROS_DEBUG_STREAM(ec_data_);

    // Match the node rate and the Ectane rate
    rate_ = acquisition_rate_;

    initHardware();

    publisher_ = nh_.advertise<eddyfi_ectane::Observation>(ns_ + "/eddy_current_data", 100, true);

    balance_service_ = nh_.advertiseService(ns_ + "/balance_device", &EctaneNode::balanceDevice, this);
    stream_service_ = nh_.advertiseService(ns_ + "/stream_data", &EctaneNode::streamData, this);

    ROS_INFO_STREAM(node_name_ << ": Node initialised");
}


// TODO: Consider moving this into the ectane_handler
EctaneNode::~EctaneNode() {
    ROS_INFO_STREAM(node_name_ << ": Shutting down...");
    ectane_handler_.disconnectDevice();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM(node_name_ << ": Shut down");
}


template <typename ParamType>
ParamType EctaneNode::paramHandler(std::string param_name, ParamType& param_value) {
    while (!nh_.hasParam(param_name) and !*kill_now_) {
        ROS_INFO_STREAM_THROTTLE(10, node_name_ << ": Waiting for parameter " << param_name);
    }
    nh_.getParam(param_name, param_value);
    ROS_DEBUG_STREAM(node_name_ << ": Read in parameter " << param_name << " = " << param_value);
    return param_value;
}


Configurations EctaneNode::configurationsFromString(std::string str) {
    if(str == "example") return Configurations::example;
    else if(str == "hf_probe_axial_1_channel") return Configurations::hf_probe_axial_1_channel;
    else if(str == "hf_probe_transversal_1_channel") return Configurations::hf_probe_transversal_1_channel;
    else if(str == "hf_probe_axial_and_transversal_1_channel") return Configurations::hf_probe_axial_and_transversal_1_channel;
    else if(str == "hf_probe_axial_and_transversal_4_channels") return Configurations::hf_probe_axial_and_transversal_4_channels;
    else if(str == "lf_probe_axial_1_channel") return Configurations::lf_probe_axial_1_channel;
    else if(str == "lf_probe_transversal_1_channel") return Configurations::lf_probe_transversal_1_channel;
    else if(str == "lf_probe_axial_and_transversal_1_channel") return Configurations::lf_probe_axial_and_transversal_1_channel;
    else if(str == "lf_probe_axial_and_transversal_4_channels") return Configurations::lf_probe_axial_and_transversal_4_channels;
    else {
        ROS_ERROR_STREAM(node_name_ << ": Failed to parse comnfiguration!");
        return Configurations::example;
    }
}


void EctaneNode::getChannelNamespaces() {
    // TODO: Bits in here that can be split out into other methods
    std::vector<std::string> param_names;
    std::vector<std::string> channels_from_geom_params;
    std::vector<std::string> channels_from_crosstalk_param;

    nh_.getParamNames(param_names);

    for (const auto& entry : param_names) {
        if (entry.find(geom_param_prefix_) != std::string::npos) {
            std::string tmp;
            tmp = entry.substr(geom_param_prefix_.size());         // Remove the namespace prefix
            tmp = tmp.substr(0, tmp.find("/") + 1);                // Remove anything under the channel namespace
            channels_from_geom_params.push_back(tmp);
        } else if (entry.find(crosstalk_param_prefix_) != std::string::npos) {
            std::string tmp;
            tmp = entry.substr(crosstalk_param_prefix_.size());    // Remove the namespace prefix
            tmp = tmp.substr(0, tmp.find("/") + 1);                // Remove anything under the channel namespace
            channels_from_crosstalk_param.push_back(tmp);
        }
    }

    uniqueElements(channels_from_geom_params);
    uniqueElements(channels_from_crosstalk_param);

    // TODO: Handle multiple frequencies
    if (channels_from_geom_params.size() != channels_from_crosstalk_param.size()) {
        ROS_ERROR_STREAM("Geometry and crosstalk parameter coil counts do not match: Check configuration and launch files");
    } else {
        for(unsigned i = 0; i < channels_from_geom_params.size(); ++i) {
            if (channels_from_geom_params[i] != channels_from_crosstalk_param[i]) {
                ROS_ERROR_STREAM("Geometry and crosstalk parameter coil counts do not match: Check configuration and launch files");
                break;
            }
        }
    }

    channel_namespaces_ = channels_from_geom_params;
}


void EctaneNode::getChannelConfigurations() {
    std::string name;
    float x;
    float y;
    int time_slot;
    int input;
    std::tuple<std::string, int, int, float, float, int, int> config_and_readings;

    for (auto& entry : channel_namespaces_) {
        name = entry.substr(0, entry.find("/"));
        EctaneNode::paramHandler(crosstalk_param_prefix_ + entry + "time_slot", time_slot);
        EctaneNode::paramHandler(crosstalk_param_prefix_ + entry + "input", input);
        EctaneNode::paramHandler(geom_param_prefix_ + entry + "x", x);
        EctaneNode::paramHandler(geom_param_prefix_ + entry + "y", y);
        config_and_readings = {
            name, 
            time_slot, 
            input, 
            x, 
            y, 
            0, 
            0
        };
        channel_configs_and_readings_.push_back(config_and_readings);
    }

    // Sort by time slot
    std::sort(begin(channel_configs_and_readings_), end(channel_configs_and_readings_), [](auto const &t1, auto const &t2) {
        return std::get<1>(t1) < std::get<1>(t2);
    });

    // Sort by input
    // TODO: Surely a better way to do this using std::sort or std::partialsort
    // TODO: Loop until sorted rather than just looping 20 times
    for(unsigned j = 1; j < 20; ++j) {
        for(unsigned i = 1; i < channel_configs_and_readings_.size(); ++i) {
            if (std::get<1>(channel_configs_and_readings_[i]) == std::get<1>(channel_configs_and_readings_[i-1])) {
                if (std::get<2>(channel_configs_and_readings_[i]) < std::get<2>(channel_configs_and_readings_[i-1])) {
                    std::iter_swap(channel_configs_and_readings_.begin() + i-1, channel_configs_and_readings_.begin() + i);
                }
            }
        }
    }
}


void EctaneNode::uniqueElements(std::vector<std::string>& vec) {
    std::sort(vec.begin(), vec.end());
    auto it = std::unique(vec.begin(), vec.end());
    vec.erase(it, vec.end());
}


void EctaneNode::initHardware() {
    ROS_INFO_STREAM(node_name_ << ": Initialising Ectane hardware...");
    float wait(1.0);

    ectane_handler_.doNothing();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(wait).sleep();

    /*
    ectane_handler_.commandSocketInitialise();
    ROS_INFO_STREAM(node_name_ << ": Command socket with Ectane initialised...");
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(wait).sleep();
    */

    ectane_handler_.connecttoDevice();
    ROS_INFO_STREAM(node_name_ << ": Connected to device...");
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(wait).sleep();

    ectane_handler_.setUpDevice();
    ROS_INFO_STREAM(node_name_ << ": Device setup...");
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(wait).sleep();

    ectane_handler_.doNothing();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(wait).sleep();

    ROS_INFO_STREAM(node_name_ << ": Ectane hardware initialised");
}


bool EctaneNode::balanceDevice(eddyfi_ectane::BalanceDevice::Request& request,
                               eddyfi_ectane::BalanceDevice::Response& response) {
    if (request.balance) {
        ROS_INFO_STREAM(node_name_ << ": Balancing device...");
        ectane_handler_.balanceDevice();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
    else {
        response.success = false;
        return true;
    }
}


bool EctaneNode::streamData(eddyfi_ectane::StreamData::Request& request,
                            eddyfi_ectane::StreamData::Response& response) {
    if (request.stream_data) {
        ROS_INFO_STREAM(node_name_ << ": Beginning data stream...");
        ectane_handler_.acquireData();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
    else {
        ROS_INFO_STREAM(node_name_ << ": Stopping data stream...");
        ectane_handler_.stopData();
        ectane_handler_.doNothing();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
}


void EctaneNode::populateEcData() {
    int i = 0;
    for (auto& entry : channel_configs_and_readings_) {
        entry = std::tuple(
            std::get<0>(entry),                                               // Channel name
            std::get<1>(entry),                                               // time_slot
            std::get<2>(entry),                                               // input
            std::get<3>(entry),                                               // x_pos
            std::get<4>(entry),                                               // y_pos
            (int32_t)(int16_t)((((*ec_data_ptr_)[i]) >> 16) & 0x0000FFFF),    // Imaginary value
            (int32_t)(int16_t)((((*ec_data_ptr_)[i]) >> 0) & 0x0000FFFF)      // Real value
            );

        ROS_DEBUG_STREAM("Buffer: " << (*ec_data_ptr_)[i]);
        ROS_DEBUG_STREAM(
            "Name: " << std::get<0>(entry) << " (r=" << std::get<6>(entry) << ",i=" << std::get<5>(entry) << ")"
            );

        //std::complex<int> reading(std::get<6>(entry), std::get<5>(entry));
        std::complex<double> reading((double)std::get<6>(entry), (double)std::get<5>(entry));

        // TODO: Need to appropriately phase rotate the reading
        double shift(3.14159 / 2);
        //double shift(0.0);
        double mag = std::abs(reading);
        double theta = std::arg(reading) + shift;
        reading = std::complex(mag * cos(theta), mag * sin(theta));

        ec_data_.channels[i].imaginary_reading = (float)std::imag(reading);
        ec_data_.channels[i].real_reading = (float)std::real(reading);

        i++;
    }
}


void EctaneNode::run() {
    while (ros::ok() and !*kill_now_) {

        // TODO: Consider running this in a dedicated thread
        while (ectane_handler_.state() == State::acquire_data) {
            ROS_INFO_STREAM_THROTTLE(60, node_name_ << ": Streaming data...");

            ectane_handler_.readData();
            ec_data_.header.stamp = ros::Time::now();
            
            populateEcData();
            publisher_.publish(ec_data_);

            ros::spinOnce();
            rate_.sleep();
        }

        ROS_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");
        ROS_INFO_STREAM_THROTTLE(60, node_name_ << ": Not streaming data...");
        ros::spinOnce();
        rate_.sleep();
    }
}


// Signal handling
bool KILL_NOW = false;


void killNow(int /*unused*/) {
    KILL_NOW = true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ectane_node");

    signal(SIGTERM, killNow);
    signal(SIGINT, killNow);
    signal(SIGHUP, killNow);

    EctaneNode ectane_node(&KILL_NOW);
    ectane_node.run();

    return 0;
}