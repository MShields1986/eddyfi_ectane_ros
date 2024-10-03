#include "ectane_nodelet.h"



namespace ectane_namespace {

EctaneNodelet::EctaneNodelet()
  : rate_(10),
    ns_("/ectane"), // TODO: Figure out how to get this when using nodelets so it isn't hardcoded
    ectane_handler_()
{
}


void EctaneNodelet::onInit()
{
    ros::NodeHandle &nh_ = getMTNodeHandle(); //getNodeHandle();
    node_name_ = getName();
    //ns_ = ros::this_node::getNamespace();
    geom_param_prefix_ = ns_ + "/geometry/";
    crosstalk_param_prefix_ = ns_ + "/crosstalk/";

    ectane_handler_.setup(
            EctaneNodelet::paramHandler(ns_ + "/settings/ectane_address", ectane_address_),
            EctaneNodelet::configurationsFromString(
                EctaneNodelet::paramHandler(ns_ + "/settings/configuration", configuration_str_)
                ),
            EctaneNodelet::paramHandler(ns_ + "/settings/acquisition_rate", acquisition_rate_),
            EctaneNodelet::paramHandler(ns_ + "/settings/gain", gain_),
            EctaneNodelet::paramHandler(ns_ + "/settings/frequency", frequency_),
            EctaneNodelet::paramHandler(ns_ + "/settings/voltage", voltage_)
       );

    ec_data_ptr_ = ectane_handler_.ec_data_ptr();

    getChannelNamespaces();
    getChannelConfigurations();

    // Populate ROS message
    //ec_data_.header.frame_id = ns_; // Include the proceeding slash...
    ec_data_.header.frame_id = "ectane";

    for (auto& entry : channel_configs_and_readings_) {
        NODELET_DEBUG_STREAM(
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

    NODELET_DEBUG_STREAM(ec_data_);

    // Match the node rate and the Ectane rate
    rate_ = acquisition_rate_;

    initHardware();

    publisher_ = nh_.advertise<eddyfi_ectane::Observation>(ns_ + "/eddy_current_data", 100, true);

    balance_service_ = nh_.advertiseService(ns_ + "/balance_device", &EctaneNodelet::balanceDevice, this);
    stream_service_ = nh_.advertiseService(ns_ + "/stream_data", &EctaneNodelet::streamData, this);

    timer_ = nh_.createTimer(ros::Duration(1 / acquisition_rate_), &EctaneNodelet::timerCb, this);

    NODELET_INFO_STREAM(node_name_ << ": Node initialised");
}


// TODO: Consider moving this into the ectane_handler
EctaneNodelet::~EctaneNodelet() {
    NODELET_INFO_STREAM(node_name_ << ": Shutting down...");
    ectane_handler_.disconnectDevice();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(0.5).sleep();
    NODELET_INFO_STREAM(node_name_ << ": Shut down");
}


template <typename ParamType>
ParamType EctaneNodelet::paramHandler(std::string param_name, ParamType& param_value) {
    while (!nh_.hasParam(param_name)) {
        NODELET_INFO_STREAM_THROTTLE(10, node_name_ << ": Waiting for parameter " << param_name);
    }
    nh_.getParam(param_name, param_value);
    NODELET_DEBUG_STREAM(node_name_ << ": Read in parameter " << param_name << " = " << param_value);
    return param_value;
}


Configurations EctaneNodelet::configurationsFromString(std::string str) {
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
        NODELET_ERROR_STREAM(node_name_ << ": Failed to parse comnfiguration!");
        return Configurations::example;
    }
}


void EctaneNodelet::getChannelNamespaces() {
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
        NODELET_ERROR_STREAM("Geometry and crosstalk parameter coil counts do not match: Check configuration and launch files");
    } else {
        for(unsigned i = 0; i < channels_from_geom_params.size(); ++i) {
            if (channels_from_geom_params[i] != channels_from_crosstalk_param[i]) {
                NODELET_ERROR_STREAM("Geometry and crosstalk parameter coil counts do not match: Check configuration and launch files");
                break;
            }
        }
    }

    channel_namespaces_ = channels_from_geom_params;
}


void EctaneNodelet::getChannelConfigurations() {
    std::string name;
    float x;
    float y;
    int time_slot;
    int input;
    std::tuple<std::string, int, int, float, float, int, int> config_and_readings;

    for (auto& entry : channel_namespaces_) {
        name = entry.substr(0, entry.find("/"));
        EctaneNodelet::paramHandler(crosstalk_param_prefix_ + entry + "time_slot", time_slot);
        EctaneNodelet::paramHandler(crosstalk_param_prefix_ + entry + "input", input);
        EctaneNodelet::paramHandler(geom_param_prefix_ + entry + "x", x);
        EctaneNodelet::paramHandler(geom_param_prefix_ + entry + "y", y);
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


void EctaneNodelet::uniqueElements(std::vector<std::string>& vec) {
    std::sort(vec.begin(), vec.end());
    auto it = std::unique(vec.begin(), vec.end());
    vec.erase(it, vec.end());
}


void EctaneNodelet::initHardware() {
    NODELET_INFO_STREAM(node_name_ << ": Initialising Ectane hardware...");
    float wait(1.0);

    ectane_handler_.doNothing();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(wait).sleep();

    /*
    ectane_handler_.commandSocketInitialise();
    NODELET_INFO_STREAM(node_name_ << ": Command socket with Ectane initialised...");
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(wait).sleep();
    */

    ectane_handler_.connecttoDevice();
    NODELET_INFO_STREAM(node_name_ << ": Connected to device...");
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(wait).sleep();

    ectane_handler_.setUpDevice();
    NODELET_INFO_STREAM(node_name_ << ": Device setup...");
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(wait).sleep();

    ectane_handler_.doNothing();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(wait).sleep();

    NODELET_INFO_STREAM(node_name_ << ": Ectane hardware initialised");
}


bool EctaneNodelet::balanceDevice(eddyfi_ectane::BalanceDevice::Request& request,
                               eddyfi_ectane::BalanceDevice::Response& response) {
    if (request.balance) {
        NODELET_INFO_STREAM(node_name_ << ": Balancing device...");
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


bool EctaneNodelet::streamData(eddyfi_ectane::StreamData::Request& request,
                            eddyfi_ectane::StreamData::Response& response) {
    if (request.stream_data) {
        NODELET_INFO_STREAM(node_name_ << ": Beginning data stream...");
        ectane_handler_.acquireData();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
    else {
        NODELET_INFO_STREAM(node_name_ << ": Stopping data stream...");
        ectane_handler_.stopData();
        ectane_handler_.doNothing();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
}


void EctaneNodelet::populateEcData() {
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

        NODELET_DEBUG_STREAM("Buffer: " << (*ec_data_ptr_)[i]);
        NODELET_DEBUG_STREAM(
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


void EctaneNodelet::timerCb(const ros::TimerEvent& event){
    NODELET_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");

    if (ectane_handler_.state() == State::acquire_data) {
        NODELET_INFO_STREAM_THROTTLE(60, node_name_ << ": Streaming data...");

        ectane_handler_.readData();
        ec_data_.header.stamp = ros::Time::now();
        
        populateEcData();
        publisher_.publish(ec_data_);
    } else {
        NODELET_INFO_STREAM_THROTTLE(60, node_name_ << ": Not streaming data...");
    }
}



} // namespace ectane_namespace

PLUGINLIB_EXPORT_CLASS(ectane_namespace::EctaneNodelet, nodelet::Nodelet);