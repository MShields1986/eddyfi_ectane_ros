#include "reconstruction_node.h"



ReconstructionNode::ReconstructionNode(bool* kill_now)
     : nh_(),
       rate_(10),
       kill_now_(kill_now),
       node_name_(ros::this_node::getName()),
       ns_(ros::this_node::getNamespace()),
       axial_subscriber_(nh_.subscribe(ns_ + "/axial_data_at_probe", 1, &ReconstructionNode::axialCallback, this)),
       transversal_subscriber_(nh_.subscribe(ns_ + "/transversal_data_at_probe", 1, &ReconstructionNode::transversalCallback, this)),
       axial_publisher_(nh_.advertise<sensor_msgs::PointCloud2>(ns_ + "/axial_reconstruction", 10, true)),
       transversal_publisher_(nh_.advertise<sensor_msgs::PointCloud2>(ns_ + "/transversal_reconstruction", 10, true)),
       recon_frame_id_("fixture_1"),  // TODO: Use a param
       tfListener_(tfBuffer_)
{
    axial_point_cloud_.header.stamp = ros::Time::now();
    axial_point_cloud_.header.frame_id = recon_frame_id_;

    sensor_msgs::PointCloud2Modifier axial_modifier(axial_point_cloud_);
    axial_modifier.setPointCloud2Fields(
        5,
        "x", 1, sensor_msgs::PointField::FLOAT32,               // 32 bits = 4 bytes
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "Imaginary", 1, sensor_msgs::PointField::FLOAT32,
        "Real", 1, sensor_msgs::PointField::FLOAT32
        );

    axial_point_cloud_.height = 1;
    axial_point_cloud_.width = 0;
    axial_point_cloud_.is_dense = true;
    axial_point_cloud_.point_step = 20;                           // 5 fields * 4 bytes per field
    axial_point_cloud_.row_step = axial_point_cloud_.point_step * axial_point_cloud_.width;
    axial_point_cloud_.data.resize(axial_point_cloud_.row_step);

    trans_point_cloud_.header.stamp = ros::Time::now();
    trans_point_cloud_.header.frame_id = recon_frame_id_;

    sensor_msgs::PointCloud2Modifier trans_modifier(trans_point_cloud_);
    trans_modifier.setPointCloud2Fields(
        5,
        "x", 1, sensor_msgs::PointField::FLOAT32,               // 32 bits = 4 bytes
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "Imaginary", 1, sensor_msgs::PointField::FLOAT32,
        "Real", 1, sensor_msgs::PointField::FLOAT32
        );

    trans_point_cloud_.height = 1;
    trans_point_cloud_.width = 0;
    trans_point_cloud_.is_dense = true;
    trans_point_cloud_.point_step = 20;                           // 5 fields * 4 bytes per field
    trans_point_cloud_.row_step = trans_point_cloud_.point_step * trans_point_cloud_.width;
    trans_point_cloud_.data.resize(trans_point_cloud_.row_step);

    trans_point_cloud_.header.frame_id = recon_frame_id_;
}


void ReconstructionNode::axialCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    axial_point_cloud_.header.stamp = msg->header.stamp;

    sensor_msgs::PointCloud2 output_pointcloud2;

    try {
        trans_ = tfBuffer_.lookupTransform(
          recon_frame_id_,
          msg->header.frame_id,
          msg->header.stamp,
          ros::Duration(1.0)
          );

        tf2::doTransform<sensor_msgs::PointCloud2>(*msg, output_pointcloud2, trans_);

        axial_point_cloud_.width += output_pointcloud2.width;
        uint64_t prev_size = axial_point_cloud_.data.size();
        axial_point_cloud_.data.resize(axial_point_cloud_.data.size() + output_pointcloud2.data.size());

        std::copy(
            output_pointcloud2.data.begin(),
            output_pointcloud2.data.end(),
            axial_point_cloud_.data.begin() + prev_size);

        axial_publisher_.publish(axial_point_cloud_);

    } catch (tf2::TransformException& ex) {
        ROS_INFO_STREAM(node_name_ << ": Could not find transform " << recon_frame_id_ << " to " + msg->header.frame_id + ": " + ex.what());
    }
}


void ReconstructionNode::transversalCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    trans_point_cloud_.header.stamp = msg->header.stamp;

    sensor_msgs::PointCloud2 output_pointcloud2;

    try {
        trans_ = tfBuffer_.lookupTransform(
          recon_frame_id_,
          msg->header.frame_id,
          msg->header.stamp,
          ros::Duration(1.0)
          );

        tf2::doTransform<sensor_msgs::PointCloud2>(*msg, output_pointcloud2, trans_);

        trans_point_cloud_.width += output_pointcloud2.width;
        uint64_t prev_size = trans_point_cloud_.data.size();
        trans_point_cloud_.data.resize(trans_point_cloud_.data.size() + output_pointcloud2.data.size());

        std::copy(
            output_pointcloud2.data.begin(),
            output_pointcloud2.data.end(),
            trans_point_cloud_.data.begin() + prev_size);

        transversal_publisher_.publish(trans_point_cloud_);

    } catch (tf2::TransformException& ex) {
        ROS_INFO_STREAM(node_name_ << ": Could not find transform " << recon_frame_id_ << " to " + msg->header.frame_id + ": " + ex.what());
    }
}


void ReconstructionNode::run() {
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
    ros::init(argc, argv, "reconstruction_node");

    signal(SIGTERM, killNow);
    signal(SIGINT, killNow);
    signal(SIGHUP, killNow);

    ReconstructionNode reconstruction_node(&KILL_NOW);
    reconstruction_node.run();

    return 0;
}