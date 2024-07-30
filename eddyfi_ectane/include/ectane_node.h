#pragma once

#include <signal.h>

#include <ros/ros.h>

#include <std_msgs/Header.h>

#include "eddyfi_ectane/Observation.h"
#include "eddyfi_ectane/TimeSlot.h"
#include "eddyfi_ectane/Channel.h"
#include "eddyfi_ectane/Frequency.h"

#include "ectane_handler.h"
#include "eddyfi_ectane/BalanceDevice.h"
#include "eddyfi_ectane/StreamData.h"


class EctaneNode
{
public:
    explicit EctaneNode(bool* kill_now);
    ~EctaneNode();

    void                        run();

private:
    template <typename ParamType>
    ParamType                   paramHandler(std::string param_name, ParamType& param_value);

    void                        initHardware();
    bool                        balanceDevice(eddyfi_ectane::BalanceDevice::Request& request,
                                              eddyfi_ectane::BalanceDevice::Response& response);
    bool                        streamData(eddyfi_ectane::StreamData::Request& request,
                                           eddyfi_ectane::StreamData::Response& response);
    void                        populateEcData();

    ros::NodeHandle             nh_;
    ros::Rate                   rate_;
    bool*                       kill_now_;
    std::string                 node_name_;
    std::string                 ns_;

    std::string                 ectane_address_;
    int32_t                     acquisition_rate_;
    int32_t                     gain_;
    int32_t                     frequency_;
    float                       voltage_;

    ros::Publisher              publisher_;

    ros::ServiceServer          balance_service_;
    ros::ServiceServer          stream_service_;

    EctaneHandler               ectane_handler_;
    const std::string*          ec_data_ptr_;
    eddyfi_ectane::Observation  ec_data_;
};
