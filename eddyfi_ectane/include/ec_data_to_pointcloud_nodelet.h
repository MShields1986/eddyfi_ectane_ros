#pragma once

#include <signal.h>
#include <algorithm>
#include <vector>

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
#include "eddyfi_ectane/Channel.h"

#include "channel_naming.h"


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
    sensor_msgs::PointCloud2          toPointCloud(const std_msgs::Header& header,
                                                   const std::vector<const eddyfi_ectane::Channel*>& channels) const;

    ros::NodeHandle                   nh_;
    std::string                       node_name_;
    std::string                       ns_;

    ros::Subscriber                   subscriber_;
    int                               num_frequencies_;
    std::vector<ros::Publisher>       axial_publishers_;          // Indexed by frequency index - 1
    std::vector<ros::Publisher>       transversal_publishers_;    // Indexed by frequency index - 1

    bool                              no_robot_;
    std::string                       no_robot_extrinsic_frame_id_;
    float                             no_robot_x_pos_increment_;
    float                             no_robot_x_pos_;
    tf2_ros::TransformBroadcaster     br_;

};

} // namespace ec_data_to_pointcloud_namespace
