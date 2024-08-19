#include "reconstruction_node.h"



ReconstructionNode::ReconstructionNode(bool* kill_now)
     : nh_(),
       rate_(10),
       kill_now_(kill_now),
       node_name_(ros::this_node::getName()),
       ns_(ros::this_node::getNamespace()),
       axial_subscriber_(nh_.subscribe(ns_ + "/axial_data_at_probe", 1, &ReconstructionNode::axialCallback, this)),
       transversal_subscriber_(nh_.subscribe(ns_ + "/transversal_data_at_probe", 1, &ReconstructionNode::transversalCallback, this)),
       axial_publisher_(nh_.advertise<sensor_msgs::PointCloud>(ns_ + "/axial_reconstruction", 10, true)),
       transversal_publisher_(nh_.advertise<sensor_msgs::PointCloud>(ns_ + "/transversal_reconstruction", 10, true)),
       recon_frame_id_("map"),  // TODO: Use a param
       tfListener_(tfBuffer_)
{
    axial_point_cloud_.header.frame_id = recon_frame_id_;
    trans_point_cloud_.header.frame_id = recon_frame_id_;

    axial_imaginary_channel_.name = "Imaginary";
    axial_real_channel_.name = "Real";

    trans_imaginary_channel_.name = "Imaginary";
    trans_real_channel_.name = "Real";
}


void ReconstructionNode::axialCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    axial_point_cloud_.header.stamp = msg->header.stamp;

    sensor_msgs::PointCloud transformed_pointcloud;

    sensor_msgs::PointCloud2 input_pointcloud2;
    sensor_msgs::PointCloud2 output_pointcloud2;

    sensor_msgs::convertPointCloudToPointCloud2(
        *msg,
        input_pointcloud2
    );

    try {
        trans_ = tfBuffer_.lookupTransform(
          recon_frame_id_,
          msg->header.frame_id,
          msg->header.stamp,
          ros::Duration(1.0)
          );

        tf2::doTransform<sensor_msgs::PointCloud2>(input_pointcloud2, output_pointcloud2, trans_);

        sensor_msgs::convertPointCloud2ToPointCloud(
            output_pointcloud2,
            transformed_pointcloud
        );

        axial_point_cloud_.points.insert(
            axial_point_cloud_.points.end(), 
            transformed_pointcloud.points.begin(), 
            transformed_pointcloud.points.end()
            );

        // TODO:: Validate channel 0 and channel 1 on their names (maybe loop over channels and if the name is Real or Imag...)
        axial_imaginary_channel_.values.insert(
            axial_imaginary_channel_.values.end(), 
            msg->channels[0].values.begin(), 
            msg->channels[0].values.end()
            );

        axial_real_channel_.values.insert(
            axial_real_channel_.values.end(), 
            msg->channels[1].values.begin(), 
            msg->channels[1].values.end()
            );

        axial_point_cloud_.channels.clear();
        axial_point_cloud_.channels.push_back(axial_imaginary_channel_);
        axial_point_cloud_.channels.push_back(axial_real_channel_);

        axial_publisher_.publish(axial_point_cloud_);

    } catch (tf2::TransformException& ex) {
        ROS_INFO_STREAM(node_name_ << ": Could not find transform " << recon_frame_id_ << " to " + msg->header.frame_id + ": " + ex.what());
    }
}


void ReconstructionNode::transversalCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    trans_point_cloud_.header.stamp = msg->header.stamp;

    sensor_msgs::PointCloud transformed_pointcloud;

    sensor_msgs::PointCloud2 input_pointcloud2;
    sensor_msgs::PointCloud2 output_pointcloud2;

    sensor_msgs::convertPointCloudToPointCloud2(
        *msg,
        input_pointcloud2
    );

    try {
        trans_ = tfBuffer_.lookupTransform(
          recon_frame_id_,
          msg->header.frame_id,
          msg->header.stamp,
          ros::Duration(1.0)
          );

        tf2::doTransform<sensor_msgs::PointCloud2>(input_pointcloud2, output_pointcloud2, trans_);

        sensor_msgs::convertPointCloud2ToPointCloud(
            output_pointcloud2,
            transformed_pointcloud
        );

        trans_point_cloud_.points.insert(
            trans_point_cloud_.points.end(), 
            transformed_pointcloud.points.begin(), 
            transformed_pointcloud.points.end()
            );

        // TODO:: Validate channel 0 and channel 1 on their names (maybe loop over channels and if the name is Real or Imag...)
        trans_imaginary_channel_.values.insert(
            trans_imaginary_channel_.values.end(), 
            msg->channels[0].values.begin(), 
            msg->channels[0].values.end()
            );

        trans_real_channel_.values.insert(
            trans_real_channel_.values.end(), 
            msg->channels[1].values.begin(), 
            msg->channels[1].values.end()
            );

        trans_point_cloud_.channels.clear();
        trans_point_cloud_.channels.push_back(trans_imaginary_channel_);
        trans_point_cloud_.channels.push_back(trans_real_channel_);

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