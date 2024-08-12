#pragma once

#include <signal.h>
#include <algorithm>

#include <ros/ros.h>

#include <std_msgs/Header.h>

#include "eddyfi_ectane/Observation.h"
#include "eddyfi_ectane/TimeSlot.h"
#include "eddyfi_ectane/Channel.h"
#include "eddyfi_ectane/Frequency.h"

#include "c_sdk_data_structures.h"
#include "ectane_handler.h"
#include "eddyfi_ectane/BalanceDevice.h"
#include "eddyfi_ectane/StreamData.h"


class EctaneNode
{
public:
    explicit EctaneNode(bool* kill_now);
    ~EctaneNode();

    void                                 run();

private:
    template <typename ParamType>
    ParamType                            paramHandler(std::string param_name, ParamType& param_value);

    Configurations                       configurationsFromString(std::string str);

    void                                 initHardware();
    bool                                 balanceDevice(eddyfi_ectane::BalanceDevice::Request& request,
                                                       eddyfi_ectane::BalanceDevice::Response& response);
    bool                                 streamData(eddyfi_ectane::StreamData::Request& request,
                                                    eddyfi_ectane::StreamData::Response& response);
    void                                 populateEcData();

    void                                 getChannelNamespaces();
    void                                 getChannelConfigurations();
    void                                 uniqueElements(std::vector<std::string>& vec);

    ros::NodeHandle                                          nh_;
    ros::Rate                                                rate_;
    bool*                                                    kill_now_;
    std::string                                              node_name_;
    std::string                                              ns_;
    std::string                                              geom_param_prefix_;
    std::string                                              crosstalk_param_prefix_;
    std::vector
        <std::tuple
            <
            std::string,     // Channel Name
            int, int,        // time_slot, input
            float, float,    // x_pos, y_pos
            int, int         // imaginary, real EC readings
            >
        >                                                    channel_configs_and_readings_;

    std::string                                              ectane_address_;
    std::string                                              configuration_str_;
    int32_t                                                  acquisition_rate_;
    int32_t                                                  gain_;
    int32_t                                                  frequency_;
    float                                                    voltage_;
    std::vector<std::string>                                 channel_namespaces_;

    ros::Publisher                                           publisher_;

    ros::ServiceServer                                       balance_service_;
    ros::ServiceServer                                       stream_service_;

    EctaneHandler                                            ectane_handler_;
    const std::string*                                       ec_data_ptr_;
    eddyfi_ectane::Observation                               ec_data_;
};
