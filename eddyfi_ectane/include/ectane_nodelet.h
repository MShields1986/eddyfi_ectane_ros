#pragma once

#include <signal.h>
#include <algorithm>
#include <atomic>
#include <complex>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Header.h>

#include "eddyfi_ectane/Observation.h"
#include "eddyfi_ectane/TimeSlot.h"
#include "eddyfi_ectane/Channel.h"
#include "eddyfi_ectane/Frequency.h"

#include "c_sdk_data_structures.h"
#include "channel_naming.h"
#include "ectane_handler.h"
#include "eddyfi_ectane/BalanceDevice.h"
#include "eddyfi_ectane/StreamData.h"


namespace ectane_namespace {

struct ChannelConfig {
    std::string name;
    int time_slot;
    int input;
    int frequency_index;     // 1-based
    float x_position;
    float y_position;
};

class EctaneNodelet : public nodelet::Nodelet
{
public:
    explicit EctaneNodelet();
    ~EctaneNodelet();

private:
    virtual void                         onInit();

    template <typename ParamType>
    ParamType                            paramHandler(std::string param_name, ParamType& param_value);

    Configurations                       configurationsFromString(std::string str);

    void                                 initHardware();
    bool                                 balanceDevice(eddyfi_ectane::BalanceDevice::Request& request,
                                                       eddyfi_ectane::BalanceDevice::Response& response);
    bool                                 streamData(eddyfi_ectane::StreamData::Request& request,
                                                    eddyfi_ectane::StreamData::Response& response);
    bool                                 populateEcData();

    void                                 getChannelNamespaces();
    void                                 getChannelConfigurations();
    bool                                 validateFrequencies();
    void                                 uniqueElements(std::vector<std::string>& vec);

    void                                 updateStreaming();
    void                                 acquisitionLoop();

    ros::NodeHandle                                          nh_;
    ros::Rate                                                rate_;
    std::string                                              node_name_;
    std::string                                              ns_;
    std::string                                              geom_param_prefix_;
    std::string                                              crosstalk_param_prefix_;
    std::vector<ChannelConfig>                               channel_configs_;     // In C SDK data stream order
    int                                                      num_frequencies_;

    std::string                                              ectane_address_;
    std::string                                              configuration_str_;
    int32_t                                                  acquisition_rate_;
    int32_t                                                  gain_;
    std::vector<int32_t>                                     frequencies_;
    std::vector<float>                                       voltages_;
    std::vector<std::string>                                 channel_namespaces_;

    ros::Publisher                                           publisher_;

    ros::ServiceServer                                       balance_service_;
    ros::ServiceServer                                       stream_service_;

    EctaneHandler                                            ectane_handler_;
    std::vector<int>                                         ec_data_raw_;
    eddyfi_ectane::Observation                               ec_data_;

    // Reading the C SDK data stream blocks, so it runs on its own thread rather than holding up a callback
    std::atomic<bool>                                        streaming_;
    std::atomic<bool>                                        stopping_;
    std::thread                                              acquisition_thread_;
};

} // namespace ectane_namespace