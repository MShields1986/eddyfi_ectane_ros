#include "ectane_nodelet.h"

#include <chrono>
#include <sstream>
#include <tuple>



namespace ectane_namespace {

namespace {

// Formats parameter values, including lists, for logging
template <typename T>
std::string paramToString(const T& value) {
    std::ostringstream ss;
    ss << value;
    return ss.str();
}

template <typename T>
std::string paramToString(const std::vector<T>& values) {
    std::ostringstream ss;
    ss << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        ss << (i ? ", " : "") << values[i];
    }
    ss << "]";
    return ss.str();
}

} // namespace

EctaneNodelet::EctaneNodelet()
  : rate_(10),
    ns_("/ectane"),
    ectane_handler_(),
    streaming_(false),
    stopping_(false)
{
}


void EctaneNodelet::onInit()
{
    ros::NodeHandle &nh_ = getMTNodeHandle(); //getNodeHandle();
    node_name_ = getName();
    ns_ = getMTNodeHandle().getNamespace();
    geom_param_prefix_ = ns_ + "/geometry/";
    crosstalk_param_prefix_ = ns_ + "/crosstalk/";

    EctaneNodelet::paramHandler(ns_ + "/settings/ectane_address", ectane_address_);
    EctaneNodelet::paramHandler(ns_ + "/settings/configuration", configuration_str_);
    EctaneNodelet::paramHandler(ns_ + "/settings/acquisition_rate", acquisition_rate_);
    EctaneNodelet::paramHandler(ns_ + "/settings/gain", gain_);
    EctaneNodelet::paramHandler(ns_ + "/settings/frequencies", frequencies_);
    EctaneNodelet::paramHandler(ns_ + "/settings/voltages", voltages_);

    getChannelNamespaces();
    getChannelConfigurations();
    const bool frequencies_valid = validateFrequencies();

    // Set up regardless so the disconnect sent on destruction is well formed
    ectane_handler_.setup(
            ectane_address_,
            EctaneNodelet::configurationsFromString(configuration_str_),
            acquisition_rate_,
            gain_,
            frequencies_,
            voltages_
       );

    if (!frequencies_valid) {
        NODELET_FATAL_STREAM(node_name_ << ": Invalid frequency configuration, not initialising the Ectane");
        return;
    }

    // Populate ROS message
    //ec_data_.header.frame_id = ns_; // Include the proceeding slash...
    ec_data_.header.frame_id = "ectane";

    for (const auto& config : channel_configs_) {
        NODELET_DEBUG_STREAM(
            "Channel: " << config.name <<
            ", time_slot: " << config.time_slot <<
            ", input: " << config.input <<
            ", frequency: " << config.frequency_index <<
            ", x: " << config.x_position <<
            ", y: " << config.y_position
            );

        eddyfi_ectane::Channel channel = eddyfi_ectane::Channel();
        channel.name = config.name;
        channel.time_slot = config.time_slot;
        channel.input = config.input;
        channel.x_position = config.x_position;
        channel.y_position = config.y_position;
        channel.frequency = frequencies_[config.frequency_index - 1];
        channel.dc_voltage = voltages_[config.frequency_index - 1];

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

    publisher_ = nh_.advertise<eddyfi_ectane::Observation>(ns_ + "/eddy_current_data", 10000, true);

    balance_service_ = nh_.advertiseService(ns_ + "/balance_device", &EctaneNodelet::balanceDevice, this);
    stream_service_ = nh_.advertiseService(ns_ + "/stream_data", &EctaneNodelet::streamData, this);

    acquisition_thread_ = std::thread(&EctaneNodelet::acquisitionLoop, this);

    NODELET_INFO_STREAM(node_name_ << ": Node initialised");
}


// TODO: Consider moving this into the ectane_handler
EctaneNodelet::~EctaneNodelet() {
    NODELET_INFO_STREAM(node_name_ << ": Shutting down...");

    stopping_ = true;
    if (acquisition_thread_.joinable()) {
        ectane_handler_.interruptReadData();
        acquisition_thread_.join();
    }

    ectane_handler_.disconnectDevice();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(0.5).sleep();
    NODELET_INFO_STREAM(node_name_ << ": Shut down");
}


template <typename ParamType>
ParamType EctaneNodelet::paramHandler(std::string param_name, ParamType& param_value) {
    while (!nh_.hasParam(param_name)) {
        NODELET_INFO_STREAM_THROTTLE(10, node_name_ << ": Waiting for parameter " << param_name);
        ros::Duration(0.1).sleep();
    }
    nh_.getParam(param_name, param_value);
    NODELET_DEBUG_STREAM(node_name_ << ": Read in parameter " << param_name << " = " << paramToString(param_value));
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
    std::vector<std::string> coils_from_geom_params;
    std::vector<std::string> channels_from_crosstalk_params;

    nh_.getParamNames(param_names);

    for (const auto& entry : param_names) {
        if (entry.find(geom_param_prefix_) != std::string::npos) {
            std::string tmp;
            tmp = entry.substr(geom_param_prefix_.size());         // Remove the namespace prefix
            tmp = tmp.substr(0, tmp.find("/") + 1);                // Remove anything under the channel namespace
            coils_from_geom_params.push_back(tmp);
        } else if (entry.find(crosstalk_param_prefix_) != std::string::npos) {
            std::string tmp;
            tmp = entry.substr(crosstalk_param_prefix_.size());    // Remove the namespace prefix
            tmp = tmp.substr(0, tmp.find("/") + 1);                // Remove anything under the channel namespace
            channels_from_crosstalk_params.push_back(tmp);
        }
    }

    uniqueElements(coils_from_geom_params);
    uniqueElements(channels_from_crosstalk_params);

    // Every frequency channel of a coil shares the coil's geometry
    std::vector<std::string> coils_from_crosstalk_params;
    for (const auto& entry : channels_from_crosstalk_params) {
        coils_from_crosstalk_params.push_back(
            channel_naming::parseChannelName(entry.substr(0, entry.find("/"))).coil + "/"
            );
    }
    uniqueElements(coils_from_crosstalk_params);

    if (coils_from_geom_params != coils_from_crosstalk_params) {
        NODELET_ERROR_STREAM("Geometry and crosstalk parameter coils do not match: Check configuration and launch files");
    }

    channel_namespaces_ = channels_from_crosstalk_params;
}


void EctaneNodelet::getChannelConfigurations() {
    for (const auto& entry : channel_namespaces_) {
        ChannelConfig config;
        config.name = entry.substr(0, entry.find("/"));
        const channel_naming::ChannelId id = channel_naming::parseChannelName(config.name);
        config.frequency_index = id.frequency_index;
        EctaneNodelet::paramHandler(crosstalk_param_prefix_ + entry + "time_slot", config.time_slot);
        EctaneNodelet::paramHandler(crosstalk_param_prefix_ + entry + "input", config.input);
        EctaneNodelet::paramHandler(geom_param_prefix_ + id.coil + "/x", config.x_position);
        EctaneNodelet::paramHandler(geom_param_prefix_ + id.coil + "/y", config.y_position);
        channel_configs_.push_back(config);
    }

    // Match the C SDK data stream, which is ordered by time slot, then input, then frequency
    std::sort(begin(channel_configs_), end(channel_configs_), [](const ChannelConfig& a, const ChannelConfig& b) {
        return std::tie(a.time_slot, a.input, a.frequency_index) < std::tie(b.time_slot, b.input, b.frequency_index);
    });

    num_frequencies_ = 0;
    for (const auto& config : channel_configs_) {
        num_frequencies_ = std::max(num_frequencies_, config.frequency_index);
    }
}


bool EctaneNodelet::validateFrequencies() {
    for (const auto& config : channel_configs_) {
        if (config.frequency_index < 1) {
            NODELET_ERROR_STREAM(node_name_ << ": Channel " << config.name << " has an invalid frequency index");
            return false;
        }
    }

    if (num_frequencies_ > kMaxFrequencies) {
        NODELET_ERROR_STREAM(node_name_ << ": Configuration uses " << num_frequencies_ <<
            " frequencies but the Ectane supports at most " << kMaxFrequencies);
        return false;
    }

    if (frequencies_.size() < (std::size_t)num_frequencies_ || voltages_.size() < (std::size_t)num_frequencies_) {
        NODELET_ERROR_STREAM(node_name_ << ": Configuration " << configuration_str_ << " uses " << num_frequencies_ <<
            " frequencies but " << frequencies_.size() << " frequencies and " << voltages_.size() <<
            " voltages are set: Check settings");
        return false;
    }

    if (frequencies_.size() > (std::size_t)num_frequencies_ || voltages_.size() > (std::size_t)num_frequencies_) {
        NODELET_INFO_STREAM(node_name_ << ": Configuration " << configuration_str_ << " uses " << num_frequencies_ <<
            " frequencies, ignoring the remaining frequencies and voltages in settings");
        frequencies_.resize(num_frequencies_);
        voltages_.resize(num_frequencies_);
    }

    NODELET_INFO_STREAM(node_name_ << ": Frequencies " << paramToString(frequencies_) <<
        " Hz at voltages " << paramToString(voltages_) << " V");
    return true;
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
        updateStreaming();
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
        updateStreaming();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
    else {
        NODELET_INFO_STREAM(node_name_ << ": Stopping data stream...");
        ectane_handler_.stopData();
        ectane_handler_.doNothing();
        updateStreaming();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
}


bool EctaneNodelet::populateEcData() {
    if (ec_data_raw_.size() < ec_data_.channels.size()) {
        NODELET_WARN_STREAM_THROTTLE(10, node_name_ << ": Received " << ec_data_raw_.size() <<
            " samples for " << ec_data_.channels.size() << " channels, skipping");
        return false;
    }

    // Channels are in the same order as the C SDK data stream
    // TODO: This would make more sense maybe if it was in the ectane_handler
    for (std::size_t i = 0; i < ec_data_.channels.size(); ++i) {
        const int32_t imaginary = (int32_t)(int16_t)(((ec_data_raw_[i]) >> 16) & 0x0000FFFF);
        const int32_t real = (int32_t)(int16_t)(((ec_data_raw_[i]) >> 0) & 0x0000FFFF);

        NODELET_DEBUG_STREAM("Buffer: " << ec_data_raw_[i]);
        NODELET_DEBUG_STREAM(
            "Name: " << ec_data_.channels[i].name << " (r=" << real << ",i=" << imaginary << ")"
            );

        std::complex<double> reading((double)real, (double)imaginary);

        // TODO: Need to appropriately phase rotate the reading
        double shift(M_PI / 2.0);
        double mag = std::abs(reading);
        double theta = std::arg(reading) + shift;
        reading = std::complex(mag * cos(theta), mag * sin(theta));

        ec_data_.channels[i].imaginary_reading = (float)std::imag(reading);
        ec_data_.channels[i].real_reading = (float)std::real(reading);
    }
    return true;
}


// Data is read while the C SDK is acquiring, as only the service callbacks change its state
void EctaneNodelet::updateStreaming() {
    streaming_ = ectane_handler_.state() == State::acquire_data;
}


void EctaneNodelet::acquisitionLoop() {
    while (!stopping_ && ros::ok()) {
        NODELET_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");

        if (!streaming_) {
            NODELET_INFO_STREAM_THROTTLE(60, node_name_ << ": Not streaming data...");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        NODELET_INFO_STREAM_THROTTLE(60, node_name_ << ": Streaming data...");

        try {
            ec_data_raw_ = ectane_handler_.readData();
        } catch (const std::exception& e) {
            NODELET_WARN_STREAM_THROTTLE(10, node_name_ << ": Data read failed: " << e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Woken by the destructor rather than by data
        if (stopping_) {
            break;
        }

        ec_data_.header.stamp = ros::Time::now();

        if (populateEcData()) {
            publisher_.publish(ec_data_);
        }
    }
}


} // namespace ectane_namespace

PLUGINLIB_EXPORT_CLASS(ectane_namespace::EctaneNodelet, nodelet::Nodelet);