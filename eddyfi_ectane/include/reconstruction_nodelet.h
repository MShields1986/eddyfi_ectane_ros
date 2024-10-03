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

#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


namespace reconstruction_namespace {

class ReconstructionNodelet : public nodelet::Nodelet
{
public:
    explicit ReconstructionNodelet();

private:
    virtual void                      onInit();

    void                              axialCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void                              transversalCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void                              timerCb(const ros::TimerEvent& event);

    ros::NodeHandle                   nh_;
    int32_t                           publish_rate_;
    std::string                       node_name_;
    std::string                       ns_;

    ros::Subscriber                   axial_subscriber_;
    ros::Subscriber                   transversal_subscriber_;

    ros::Publisher                    axial_publisher_;
    ros::Publisher                    transversal_publisher_;

    ros::Timer                        timer_;

    sensor_msgs::PointCloud2          axial_point_cloud_; // TODO: Consider heap allocation
    sensor_msgs::PointCloud2          trans_point_cloud_; // TODO: Consider heap allocation

    std::string                       recon_frame_id_;

    // TF
    tf2_ros::Buffer                         tfBuffer_;
    tf2_ros::TransformListener              tfListener_;
    geometry_msgs::TransformStamped         trans_;
};

} // namespace reconstruction_namespace