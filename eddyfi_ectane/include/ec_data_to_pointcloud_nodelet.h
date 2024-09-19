#pragma once

#include <signal.h>
#include <algorithm>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "eddyfi_ectane/Observation.h"


namespace ec_data_to_pointcloud_namespace {

class EcDataToPointCloudNodelet : public nodelet::Nodelet
{
public:
    explicit EcDataToPointCloudNodelet();

private:
    virtual void                      onInit();

    template <typename ParamType>
    void                              paramHandler(std::string param_name, ParamType& param_value);

    void                              ectaneCallback(const eddyfi_ectane::Observation::ConstPtr& msg);
    void                              updateNoRobotTf(const eddyfi_ectane::Observation::ConstPtr& msg);

    ros::NodeHandle                   nh_;
    std::string                       node_name_;
    std::string                       ns_;

    ros::Subscriber                   subscriber_;
    ros::Publisher                    axial_publisher_;
    ros::Publisher                    transversal_publisher_;

    bool                              no_robot_;
    std::string                       no_robot_extrinsic_frame_id_;
    float                             no_robot_x_pos_incrememnt_;
    float                             no_robot_x_pos_;
    tf2_ros::TransformBroadcaster     br_;

};

} // namespace ec_data_to_pointcloud_namespace
