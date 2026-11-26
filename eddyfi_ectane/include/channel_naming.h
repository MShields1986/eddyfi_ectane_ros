#pragma once

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

#include <ros/ros.h>


// Crosstalk channels in multi-frequency configurations are named F<n><coil>, e.g. F2A05 is coil A05
// at the configuration's second frequency. Names without the prefix, e.g. A05, are at the first frequency.
namespace channel_naming {

struct ChannelId {
    std::string coil;        // Geometry key of the coil
    int frequency_index;     // 1-based, i.e. frequency_<n> of the C SDK command packet
};


inline ChannelId parseChannelName(const std::string& name) {
    std::size_t coil_start = 1;
    if (name.size() > 1 && name[0] == 'F') {
        while (coil_start < name.size() && std::isdigit(static_cast<unsigned char>(name[coil_start]))) {
            ++coil_start;
        }
        if (coil_start > 1 && coil_start < name.size()) {
            return {name.substr(coil_start), std::stoi(name.substr(1, coil_start - 1))};
        }
    }
    return {name, 1};
}


// Topic namespace for a frequency channel, empty for single frequency configurations so that their topic
// names are unchanged
inline std::string frequencyNamespace(int frequency_index, int num_frequencies) {
    return num_frequencies > 1 ? "frequency_" + std::to_string(frequency_index) + "/" : "";
}


// Number of frequency channels in the crosstalk configuration, waiting for it to be loaded
inline int waitForNumFrequencies(const std::string& crosstalk_param_prefix) {
    while (ros::ok()) {
        std::vector<std::string> param_names;
        ros::param::getParamNames(param_names);

        int num_frequencies = 0;
        for (const auto& param_name : param_names) {
            if (param_name.compare(0, crosstalk_param_prefix.size(), crosstalk_param_prefix) == 0) {
                std::string channel = param_name.substr(crosstalk_param_prefix.size());
                channel = channel.substr(0, channel.find("/"));
                num_frequencies = std::max(num_frequencies, parseChannelName(channel).frequency_index);
            }
        }

        if (num_frequencies > 0) {
            return num_frequencies;
        }

        ROS_INFO_STREAM_THROTTLE(10, "Waiting for crosstalk parameters under " << crosstalk_param_prefix);
        ros::Duration(0.1).sleep();
    }
    return 0;
}

} // namespace channel_naming
