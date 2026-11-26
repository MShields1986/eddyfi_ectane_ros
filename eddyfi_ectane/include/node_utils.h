#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "channel_naming.h"


namespace node_utils {

// Parameters come from the launch file's YAML, there is no parameter server to wait on as in ROS 1
inline rclcpp::Parameter requiredParam(const rclcpp::Node& node, const std::string& name) {
    rclcpp::Parameter parameter;
    if (!node.get_parameter(name, parameter)) {
        throw std::runtime_error(std::string(node.get_name()) + ": Missing parameter " + name +
            ": Check configuration and launch files");
    }
    return parameter;
}


// YAML numbers without a decimal point load as integers, so accept either for floating point values
inline double asDouble(const rclcpp::Parameter& parameter) {
    switch (parameter.get_type()) {
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return static_cast<double>(parameter.as_int());
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            return parameter.as_double();
        default:
            throw std::runtime_error("Parameter " + parameter.get_name() + " must be a number");
    }
}


inline std::vector<double> asDoubleArray(const rclcpp::Parameter& parameter) {
    switch (parameter.get_type()) {
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
            const std::vector<int64_t> values = parameter.as_integer_array();
            return std::vector<double>(values.begin(), values.end());
        }
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            return parameter.as_double_array();
        default:
            throw std::runtime_error("Parameter " + parameter.get_name() + " must be a list of numbers");
    }
}


// Number of frequency channels in the crosstalk configuration loaded into the node
inline int numFrequencies(const rclcpp::Node& node) {
    const std::vector<std::string> channels =
        channel_naming::namesUnder(node.list_parameters({"crosstalk"}, 0).names, "crosstalk.");
    const int num_frequencies = channel_naming::numFrequencies(channels);
    if (num_frequencies < 1) {
        throw std::runtime_error(std::string(node.get_name()) +
            ": No crosstalk parameters: Check configuration and launch files");
    }
    return num_frequencies;
}


// Eddy current data and the clouds at the probe, holding up to 10 s of observations at 100 Hz
inline rclcpp::QoS dataQoS() {
    return rclcpp::QoS(rclcpp::KeepLast(1000)).reliable();
}


// Reconstructions are republished whole, so late subscribers only need the latest one
inline rclcpp::QoS reconstructionQoS() {
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

} // namespace node_utils
