#pragma once

#include <signal.h>
#include <algorithm>
#include <deque>

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
    virtual void                            onInit();
    template <typename ParamType>
    ParamType                               paramHandler(std::string param_name, ParamType& param_value);
    void                                    axialCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void                                    transversalCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void                                    timerProcCb(const ros::TimerEvent& event);
    void                                    timerPubCb(const ros::TimerEvent& event);
    void                                    updateReconCloud(std::deque<sensor_msgs::PointCloud2>& buffer,
                                                             sensor_msgs::PointCloud2& recon_cloud);

    ros::NodeHandle                         nh_;
    float                                   process_rate_;
    float                                   publish_rate_;
    std::string                             node_name_;
    std::string                             ns_;

    ros::Subscriber                         axial_subscriber_;
    ros::Subscriber                         transversal_subscriber_;

    ros::Publisher                          axial_publisher_;
    ros::Publisher                          transversal_publisher_;

    ros::Timer                              proc_timer_;
    ros::Timer                              pub_timer_;

    sensor_msgs::PointCloud2                axial_point_cloud_;
    sensor_msgs::PointCloud2                trans_point_cloud_;
    std::deque<sensor_msgs::PointCloud2>    axial_buffer_;
    std::deque<sensor_msgs::PointCloud2>    trans_buffer_;

    std::string                             recon_frame_id_;

    // TF
    tf2_ros::Buffer                         tfBuffer_;
    tf2_ros::TransformListener              tfListener_;
    geometry_msgs::TransformStamped         trans_;
};

} // namespace reconstruction_namespace