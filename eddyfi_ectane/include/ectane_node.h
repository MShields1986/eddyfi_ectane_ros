#pragma once

#include <signal.h>

#include <ros/ros.h>

#include "ectane_handler.h"


class EctaneNode
{
public:
    explicit EctaneNode(bool* kill_now);
    void run();

private:
    ros::NodeHandle             nh_;
    ros::Rate                   rate_;
    bool*                       kill_now_;
    std::string                 node_name_;
    std::string                 ns_;

    EctaneHandler               ectane_handler_;
    //ros::Publisher              publisher_;
};
