#pragma once

#include <signal.h>
#include <algorithm>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

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

#include "channel_naming.h"


namespace reconstruction_namespace {

struct BufferedCloud {
    ros::Time                               received;         // Local time, independent of the publisher's clock
    sensor_msgs::PointCloud2                cloud;
};

// Reconstruction of one coil orientation (axial or transversal) at one frequency
struct ReconstructionStream {
    ros::Subscriber                         subscriber;
    ros::Publisher                          publisher;

    std::deque<BufferedCloud>               buffer;           // Guarded by buffer_mutex
    std::mutex                              buffer_mutex;

    sensor_msgs::PointCloud2                recon_cloud;      // Guarded by recon_mutex
    std::mutex                              recon_mutex;
};

class ReconstructionNodelet : public nodelet::Nodelet
{
public:
    explicit ReconstructionNodelet();

private:
    virtual void                            onInit();
    template <typename ParamType>
    ParamType                               paramHandler(std::string param_name, ParamType& param_value);
    void                                    addStream(ros::NodeHandle& nh, const std::string& topic_prefix,
                                                      const std::string& orientation);
    void                                    timerProcCb(const ros::TimerEvent& event);
    void                                    timerPubCb(const ros::TimerEvent& event);
    void                                    updateReconCloud(ReconstructionStream& stream);

    ros::NodeHandle                         nh_;
    float                                   process_rate_;
    float                                   publish_rate_;
    std::string                             node_name_;
    std::string                             ns_;

    // Axial and transversal streams for each frequency
    std::vector<std::unique_ptr<ReconstructionStream>> streams_;

    ros::Timer                              proc_timer_;
    ros::Timer                              pub_timer_;

    std::string                             recon_frame_id_;

    // TF
    tf2_ros::Buffer                         tfBuffer_;
    tf2_ros::TransformListener              tfListener_;
    geometry_msgs::TransformStamped         trans_;
};

} // namespace reconstruction_namespace