#pragma once

#include <signal.h>

#include <ros/ros.h>

// TODO: Remove Twist once custom message is made
#include <geometry_msgs/Twist.h>

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
    void                        initHardware();
    bool                        balanceDevice(eddyfi_ectane::BalanceDevice::Request& request,
                                              eddyfi_ectane::BalanceDevice::Response& response);
    bool                        streamData(eddyfi_ectane::StreamData::Request& request,
                                           eddyfi_ectane::StreamData::Response& response);

    ros::NodeHandle             nh_;
    ros::Rate                   rate_;
    bool*                       kill_now_;
    std::string                 node_name_;
    std::string                 ns_;

    ros::Publisher              publisher_;

    ros::ServiceServer          balance_service_;
    ros::ServiceServer          stream_service_;

    EctaneHandler               ectane_handler_;
    //ros::Publisher              publisher_;
};
