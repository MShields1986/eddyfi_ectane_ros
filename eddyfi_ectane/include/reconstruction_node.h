#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "channel_naming.h"


namespace reconstruction_namespace {

struct BufferedCloud {
    rclcpp::Time                            received;         // Local time, independent of the publisher's clock
    sensor_msgs::msg::PointCloud2           cloud;
};

// Reconstruction of one coil orientation (axial or transversal) at one frequency
struct ReconstructionStream {
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    publisher;

    std::deque<BufferedCloud>               buffer;           // Guarded by buffer_mutex
    std::mutex                              buffer_mutex;

    sensor_msgs::msg::PointCloud2           recon_cloud;      // Guarded by recon_mutex
    std::mutex                              recon_mutex;
};

class ReconstructionNode : public rclcpp::Node
{
public:
    explicit ReconstructionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void                                    addStream(const std::string& topic_prefix, const std::string& orientation);
    void                                    timerProcCb();
    void                                    timerPubCb();
    void                                    updateReconCloud(ReconstructionStream& stream);

    double                                  process_rate_;
    double                                  publish_rate_;
    std::string                             recon_frame_id_;

    // Subscriptions only buffer, so may run concurrently, while each timer runs one at a time
    rclcpp::CallbackGroup::SharedPtr        subscriber_group_;
    rclcpp::CallbackGroup::SharedPtr        proc_group_;
    rclcpp::CallbackGroup::SharedPtr        pub_group_;

    // Axial and transversal streams for each frequency
    std::vector<std::unique_ptr<ReconstructionStream>> streams_;

    rclcpp::TimerBase::SharedPtr            proc_timer_;
    rclcpp::TimerBase::SharedPtr            pub_timer_;

    // TF
    std::unique_ptr<tf2_ros::Buffer>        tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;
    geometry_msgs::msg::TransformStamped    trans_;
};

} // namespace reconstruction_namespace
