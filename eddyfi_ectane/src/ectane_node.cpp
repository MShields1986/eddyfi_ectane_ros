#include "ectane_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

#include "node_utils.h"



namespace ectane_namespace {

namespace {

// Formats values, including lists, for logging
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

EctaneNode::EctaneNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("ectane_handler",
                 rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)),
    num_frequencies_(0),
    streaming_(false),
    stopping_(false)
{
    ectane_address_ = node_utils::requiredParam(*this, "ectane_address").as_string();
    configuration_str_ = node_utils::requiredParam(*this, "configuration").as_string();
    acquisition_rate_ = node_utils::requiredParam(*this, "acquisition_rate").as_int();
    gain_ = node_utils::requiredParam(*this, "gain").as_int();
    const rclcpp::Parameter frequencies = node_utils::requiredParam(*this, "frequencies");
    for (const int64_t frequency : frequencies.as_integer_array()) {
        frequencies_.push_back(static_cast<int32_t>(frequency));
    }
    for (const double voltage : node_utils::asDoubleArray(node_utils::requiredParam(*this, "voltages"))) {
        voltages_.push_back(static_cast<float>(voltage));
    }

    getChannelNames();
    getChannelConfigurations();
    validateFrequencies();

    // Hosts the server that the C SDK connects to for commands, blocking until it does
    RCLCPP_INFO(get_logger(), "Waiting for the C SDK to connect...");
    ectane_handler_ = std::make_unique<EctaneHandler>();
    ectane_handler_->setup(
            ectane_address_,
            configurationsFromString(configuration_str_),
            acquisition_rate_,
            gain_,
            frequencies_,
            voltages_
       );

    // Populate ROS message
    observation_template_.header.frame_id = "ectane";

    for (const auto& config : channel_configs_) {
        RCLCPP_DEBUG_STREAM(get_logger(),
            "Channel: " << config.name <<
            ", time_slot: " << config.time_slot <<
            ", input: " << config.input <<
            ", frequency: " << config.frequency_index <<
            ", x: " << config.x_position <<
            ", y: " << config.y_position
            );

        eddyfi_ectane::msg::Channel channel;
        channel.name = config.name;
        channel.time_slot = config.time_slot;
        channel.input = config.input;
        channel.x_position = config.x_position;
        channel.y_position = config.y_position;
        channel.frequency = frequencies_[config.frequency_index - 1];
        channel.dc_voltage = voltages_[config.frequency_index - 1];

        observation_template_.channels.push_back(channel);
    }

    RCLCPP_DEBUG_STREAM(get_logger(), eddyfi_ectane::msg::to_yaml(observation_template_));

    initHardware();

    publisher_ = create_publisher<eddyfi_ectane::msg::Observation>("eddy_current_data", node_utils::dataQoS());

    balance_service_ = create_service<eddyfi_ectane::srv::BalanceDevice>(
        "balance_device",
        [this](const std::shared_ptr<eddyfi_ectane::srv::BalanceDevice::Request> request,
               std::shared_ptr<eddyfi_ectane::srv::BalanceDevice::Response> response) {
            balanceDevice(*request, *response);
        });
    stream_service_ = create_service<eddyfi_ectane::srv::StreamData>(
        "stream_data",
        [this](const std::shared_ptr<eddyfi_ectane::srv::StreamData::Request> request,
               std::shared_ptr<eddyfi_ectane::srv::StreamData::Response> response) {
            streamData(*request, *response);
        });

    acquisition_thread_ = std::thread(&EctaneNode::acquisitionLoop, this);

    // Disconnect while rclcpp shuts down, as ros2 launch passes on Ctrl-C as a second SIGINT, which kills the
    // process if it arrives once shutdown is over and the destructor is still running
    pre_shutdown_handle_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
        [this]() { shutdown(); });

    RCLCPP_INFO(get_logger(), "Node initialised");
}


EctaneNode::~EctaneNode() {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(pre_shutdown_handle_);
    shutdown();
}


// TODO: Consider moving this into the ectane_handler
void EctaneNode::shutdown() {
    std::call_once(shutdown_once_, [this]() {
        RCLCPP_INFO(get_logger(), "Shutting down...");

        stopping_ = true;
        if (acquisition_thread_.joinable()) {
            ectane_handler_->interruptReadData();
            acquisition_thread_.join();
        }

        ectane_handler_->disconnectDevice();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        RCLCPP_INFO(get_logger(), "Shut down");
    });
}


Configurations EctaneNode::configurationsFromString(const std::string& str) {
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
        RCLCPP_ERROR(get_logger(), "Failed to parse comnfiguration!");
        return Configurations::example;
    }
}


void EctaneNode::getChannelNames() {
    const std::vector<std::string> param_names = list_parameters({"geometry", "crosstalk"}, 0).names;
    const std::vector<std::string> coils_from_geom_params = channel_naming::namesUnder(param_names, "geometry.");
    channel_names_ = channel_naming::namesUnder(param_names, "crosstalk.");

    if (channel_names_.empty()) {
        throw std::runtime_error("No crosstalk parameters: Check configuration and launch files");
    }

    // Every frequency channel of a coil shares the coil's geometry
    std::vector<std::string> coils_from_crosstalk_params;
    for (const auto& name : channel_names_) {
        coils_from_crosstalk_params.push_back(channel_naming::parseChannelName(name).coil);
    }
    std::sort(coils_from_crosstalk_params.begin(), coils_from_crosstalk_params.end());
    coils_from_crosstalk_params.erase(
        std::unique(coils_from_crosstalk_params.begin(), coils_from_crosstalk_params.end()),
        coils_from_crosstalk_params.end());

    if (coils_from_geom_params != coils_from_crosstalk_params) {
        RCLCPP_ERROR(get_logger(), "Geometry and crosstalk parameter coils do not match: Check configuration and launch files");
    }
}


void EctaneNode::getChannelConfigurations() {
    for (const auto& name : channel_names_) {
        ChannelConfig config;
        config.name = name;
        const channel_naming::ChannelId id = channel_naming::parseChannelName(name);
        config.frequency_index = id.frequency_index;
        config.time_slot = node_utils::requiredParam(*this, "crosstalk." + name + ".time_slot").as_int();
        config.input = node_utils::requiredParam(*this, "crosstalk." + name + ".input").as_int();
        config.x_position = node_utils::asDouble(node_utils::requiredParam(*this, "geometry." + id.coil + ".x"));
        config.y_position = node_utils::asDouble(node_utils::requiredParam(*this, "geometry." + id.coil + ".y"));
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


void EctaneNode::validateFrequencies() {
    for (const auto& config : channel_configs_) {
        if (config.frequency_index < 1) {
            throw std::runtime_error("Channel " + config.name + " has an invalid frequency index");
        }
    }

    if (num_frequencies_ > kMaxFrequencies) {
        throw std::runtime_error("Configuration uses " + std::to_string(num_frequencies_) +
            " frequencies but the Ectane supports at most " + std::to_string(kMaxFrequencies));
    }

    if (frequencies_.size() < (std::size_t)num_frequencies_ || voltages_.size() < (std::size_t)num_frequencies_) {
        throw std::runtime_error("Configuration " + configuration_str_ + " uses " + std::to_string(num_frequencies_) +
            " frequencies but " + std::to_string(frequencies_.size()) + " frequencies and " +
            std::to_string(voltages_.size()) + " voltages are set: Check settings");
    }

    if (frequencies_.size() > (std::size_t)num_frequencies_ || voltages_.size() > (std::size_t)num_frequencies_) {
        RCLCPP_INFO_STREAM(get_logger(), "Configuration " << configuration_str_ << " uses " << num_frequencies_ <<
            " frequencies, ignoring the remaining frequencies and voltages in settings");
        frequencies_.resize(num_frequencies_);
        voltages_.resize(num_frequencies_);
    }

    RCLCPP_INFO_STREAM(get_logger(), "Frequencies " << paramToString(frequencies_) <<
        " Hz at voltages " << paramToString(voltages_) << " V");
}


void EctaneNode::initHardware() {
    RCLCPP_INFO(get_logger(), "Initialising Ectane hardware...");
    const auto wait = std::chrono::seconds(1);

    ectane_handler_->doNothing();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    std::this_thread::sleep_for(wait);

    /*
    ectane_handler_->commandSocketInitialise();
    RCLCPP_INFO(get_logger(), "Command socket with Ectane initialised...");
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    std::this_thread::sleep_for(wait);
    */

    ectane_handler_->connecttoDevice();
    RCLCPP_INFO(get_logger(), "Connected to device...");
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    std::this_thread::sleep_for(wait);

    ectane_handler_->setUpDevice();
    RCLCPP_INFO(get_logger(), "Device setup...");
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    std::this_thread::sleep_for(wait);

    ectane_handler_->doNothing();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    std::this_thread::sleep_for(wait);

    RCLCPP_INFO(get_logger(), "Ectane hardware initialised");
}


void EctaneNode::balanceDevice(const eddyfi_ectane::srv::BalanceDevice::Request& request,
                               eddyfi_ectane::srv::BalanceDevice::Response& response) {
    if (request.balance) {
        RCLCPP_INFO(get_logger(), "Balancing device...");
        ectane_handler_->balanceDevice();
        updateStreaming();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
    }
    else {
        response.success = false;
    }
}


void EctaneNode::streamData(const eddyfi_ectane::srv::StreamData::Request& request,
                            eddyfi_ectane::srv::StreamData::Response& response) {
    if (request.stream_data) {
        RCLCPP_INFO(get_logger(), "Beginning data stream...");
        ectane_handler_->acquireData();
        updateStreaming();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
    }
    else {
        RCLCPP_INFO(get_logger(), "Stopping data stream...");
        ectane_handler_->stopData();
        ectane_handler_->doNothing();
        updateStreaming();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
    }
}


// Data is read while the C SDK is acquiring, as only the service callbacks change its state
void EctaneNode::updateStreaming() {
    streaming_ = ectane_handler_->state() == State::acquire_data;
}


void EctaneNode::acquisitionLoop() {
    while (!stopping_) {
        RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 600000, "Node running");

        if (!streaming_) {
            RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 60000, "Not streaming data...");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 60000, "Streaming data...");

        try {
            ec_data_raw_ = ectane_handler_->readData();
        } catch (const std::exception& e) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Data read failed: " << e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Woken by the destructor rather than by data
        if (stopping_) {
            break;
        }

        // Published by moving, so composed subscribers in the same container receive it without a copy
        auto observation = std::make_unique<eddyfi_ectane::msg::Observation>(observation_template_);
        observation->header.stamp = now();

        if (populateEcData(*observation)) {
            publisher_->publish(std::move(observation));
        }
    }
}


bool EctaneNode::populateEcData(eddyfi_ectane::msg::Observation& observation) {
    if (ec_data_raw_.size() < observation.channels.size()) {
        RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Received " << ec_data_raw_.size() <<
            " samples for " << observation.channels.size() << " channels, skipping");
        return false;
    }

    // Channels are in the same order as the C SDK data stream
    // TODO: This would make more sense maybe if it was in the ectane_handler
    for (std::size_t i = 0; i < observation.channels.size(); ++i) {
        const int32_t imaginary = (int32_t)(int16_t)(((ec_data_raw_[i]) >> 16) & 0x0000FFFF);
        const int32_t real = (int32_t)(int16_t)(((ec_data_raw_[i]) >> 0) & 0x0000FFFF);

        RCLCPP_DEBUG_STREAM(get_logger(), "Buffer: " << ec_data_raw_[i]);
        RCLCPP_DEBUG_STREAM(get_logger(),
            "Name: " << observation.channels[i].name << " (r=" << real << ",i=" << imaginary << ")"
            );

        std::complex<double> reading((double)real, (double)imaginary);

        // TODO: Need to appropriately phase rotate the reading
        double shift(M_PI / 2.0);
        double mag = std::abs(reading);
        double theta = std::arg(reading) + shift;
        reading = std::complex(mag * cos(theta), mag * sin(theta));

        observation.channels[i].imaginary_reading = (float)std::imag(reading);
        observation.channels[i].real_reading = (float)std::real(reading);
    }
    return true;
}


} // namespace ectane_namespace

RCLCPP_COMPONENTS_REGISTER_NODE(ectane_namespace::EctaneNode)
