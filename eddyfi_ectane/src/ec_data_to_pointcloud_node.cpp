#include "ec_data_to_pointcloud_node.h"



EcDataToPointCloudNode::EcDataToPointCloudNode(bool* kill_now)
     : nh_(),
       rate_(10),
       kill_now_(kill_now),
       node_name_(ros::this_node::getName()),
       ns_(ros::this_node::getNamespace()),
       subscriber_(nh_.subscribe(ns_ + "/eddy_current_data", 1, &EcDataToPointCloudNode::ectaneCallback, this)),
       axial_publisher_(nh_.advertise<sensor_msgs::PointCloud>(ns_ + "/axial_data_at_probe", 10, true)),
       transversal_publisher_(nh_.advertise<sensor_msgs::PointCloud>(ns_ + "/transversal_data_at_probe", 10, true)),
       no_robot_(false),
       no_robot_extrinsic_frame_id_("map"),
       no_robot_x_pos_incrememnt_(0.0),
       no_robot_x_pos_(0.0)
{
    EcDataToPointCloudNode::paramHandler(ns_ + "/settings/no_robot", no_robot_);
    EcDataToPointCloudNode::paramHandler(ns_ + "/settings/no_robot_extrinsic_frame_id", no_robot_extrinsic_frame_id_);
    EcDataToPointCloudNode::paramHandler(ns_ + "/settings/no_robot_x_pos_incrememnt", no_robot_x_pos_incrememnt_);
}


template <typename ParamType>
void EcDataToPointCloudNode::paramHandler(std::string param_name, ParamType& param_value) {
    while (!nh_.hasParam(param_name) and !*kill_now_) {
        ROS_INFO_STREAM_THROTTLE(10, node_name_ << ": Waiting for parameter " << param_name);
        ros::spinOnce();
        rate_.sleep();
    }
    nh_.getParam(param_name, param_value);
    ROS_DEBUG_STREAM(node_name_ << ": Read in parameter " << param_name << " = " << param_value);
    //return param_value;
}


void EcDataToPointCloudNode::ectaneCallback(const eddyfi_ectane::Observation::ConstPtr& msg) {
    // Update dummy TF broadcaster
    if (no_robot_) {
        updateNoRobotTf(msg);
    }

    // Axial setup
    sensor_msgs::PointCloud axial_point_cloud;
    axial_point_cloud.header.stamp = msg->header.stamp;
    axial_point_cloud.header.frame_id = msg->header.frame_id;

    sensor_msgs::ChannelFloat32 axial_imaginary_channel;
    sensor_msgs::ChannelFloat32 axial_real_channel;
    axial_imaginary_channel.name = "Imaginary";
    axial_real_channel.name = "Real";

    // Transversal setup
    sensor_msgs::PointCloud trans_point_cloud;
    trans_point_cloud.header.stamp = msg->header.stamp;
    trans_point_cloud.header.frame_id = msg->header.frame_id;

    sensor_msgs::ChannelFloat32 trans_imaginary_channel;
    sensor_msgs::ChannelFloat32 trans_real_channel;
    trans_imaginary_channel.name = "Imaginary";
    trans_real_channel.name = "Real";

    for (const auto& channel : msg->channels) {
        if (channel.name.find("A") != std::string::npos) {
            geometry_msgs::Point32 point;
            point.x = channel.x_position / 1000;
            point.y = channel.y_position / 1000;
            point.z = 0.0;
            axial_point_cloud.points.push_back(point);
            axial_imaginary_channel.values.push_back(channel.imaginary_reading);
            axial_real_channel.values.push_back(channel.real_reading);
        } else if (channel.name.find("T") != std::string::npos) {
            geometry_msgs::Point32 point;
            point.x = channel.x_position / 1000;
            point.y = channel.y_position / 1000;
            point.z = 0.0;
            trans_point_cloud.points.push_back(point);

            trans_imaginary_channel.values.push_back(channel.imaginary_reading);
            trans_real_channel.values.push_back(channel.real_reading);
        }
    }

    axial_point_cloud.channels.push_back(axial_imaginary_channel);
    axial_point_cloud.channels.push_back(axial_real_channel);
    axial_publisher_.publish(axial_point_cloud);

    trans_point_cloud.channels.push_back(trans_imaginary_channel);
    trans_point_cloud.channels.push_back(trans_real_channel);
    transversal_publisher_.publish(trans_point_cloud);
}


void EcDataToPointCloudNode::updateNoRobotTf(const eddyfi_ectane::Observation::ConstPtr& msg) {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = no_robot_extrinsic_frame_id_;
    transformStamped.child_frame_id = msg->header.frame_id;
    transformStamped.transform.translation.x = no_robot_x_pos_;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br_.sendTransform(transformStamped);

    no_robot_x_pos_ += no_robot_x_pos_incrememnt_;
}


void EcDataToPointCloudNode::run() {
    while (ros::ok() and !*kill_now_) {
        ROS_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");
        ros::spinOnce();
        rate_.sleep();
    }
}


// Signal handling
bool KILL_NOW = false;


void killNow(int /*unused*/) {
    KILL_NOW = true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ec_data_to_pointcloud_node");

    signal(SIGTERM, killNow);
    signal(SIGINT, killNow);
    signal(SIGHUP, killNow);

    EcDataToPointCloudNode ec_data_to_pointcloud_node(&KILL_NOW);
    ec_data_to_pointcloud_node.run();

    return 0;
}