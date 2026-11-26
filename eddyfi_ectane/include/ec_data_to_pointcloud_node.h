#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "eddyfi_ectane/msg/observation.hpp"
#include "eddyfi_ectane/msg/channel.hpp"

#include "channel_naming.h"


namespace ec_data_to_pointcloud_namespace {

class EcDataToPointCloudNode : public rclcpp::Node
{
public:
    explicit EcDataToPointCloudNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void                              ectaneCallback(const eddyfi_ectane::msg::Observation::ConstSharedPtr msg);
    void                              updateNoRobotTf(const eddyfi_ectane::msg::Observation& msg);
    sensor_msgs::msg::PointCloud2     toPointCloud(const std_msgs::msg::Header& header,
                                                   const std::vector<const eddyfi_ectane::msg::Channel*>& channels) const;

    rclcpp::Subscription<eddyfi_ectane::msg::Observation>::SharedPtr       subscriber_;
    int                                                                     num_frequencies_;
    // Indexed by frequency index - 1
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> axial_publishers_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> transversal_publishers_;

    bool                              no_robot_;
    std::string                       no_robot_extrinsic_frame_id_;
    double                            no_robot_x_pos_increment_;
    double                            no_robot_x_pos_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

};

} // namespace ec_data_to_pointcloud_namespace
