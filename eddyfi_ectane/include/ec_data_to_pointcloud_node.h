#pragma once

#include <signal.h>
#include <algorithm>

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud.h>

#include "eddyfi_ectane/Observation.h"

class EcDataToPointCloudNode
{
public:
    explicit EcDataToPointCloudNode(bool* kill_now);

    void                              run();

private:
    void                              ectaneCallback(const eddyfi_ectane::Observation::ConstPtr& msg);

    ros::NodeHandle                   nh_;
    ros::Rate                         rate_;
    bool*                             kill_now_;
    std::string                       node_name_;
    std::string                       ns_;

    ros::Subscriber                   subscriber_;
    ros::Publisher                    axial_publisher_;
    ros::Publisher                    transversal_publisher_;

    float                             x_; // TODO: Delete in final version
};