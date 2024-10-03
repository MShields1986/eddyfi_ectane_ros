#include "reconstruction_nodelet.h"



namespace reconstruction_namespace {

ReconstructionNodelet::ReconstructionNodelet()
 :  ns_("/ectane"), // TODO: Figure out how to get this when using nodelets so it isn't hardcoded
    tfListener_(tfBuffer_)
{
}


void ReconstructionNodelet::onInit()
{
    ros::NodeHandle &nh_ = getMTNodeHandle(); //getNodeHandle();
    node_name_ = getName();
    //ns_ = ros::this_node::getNamespace();

    // Work around to allow concurrent callbacks on the same topic
    // By default you there is no concurrency on callbacks for each topic!
    //ros::SubscribeOptions ops;
    //ops.template init<std_msgs::String>(ns_ + "/axial_data_at_probe", 100, &ReconstructionNodelet::axialCallback, this);
    //ops.allow_concurrent_callbacks = true;
    //axial_subscriber_ = nh_.subscribe(ops);

    axial_subscriber_ = nh_.subscribe(ns_ + "/axial_data_at_probe", 100, &ReconstructionNodelet::axialCallback, this);
    transversal_subscriber_ = nh_.subscribe(ns_ + "/transversal_data_at_probe", 100, &ReconstructionNodelet::transversalCallback, this);

    axial_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(ns_ + "/axial_reconstruction", 10, true);
    transversal_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(ns_ + "/transversal_reconstruction", 10, true);

    // TODO: Param for the publish rate
    timer_ = nh_.createTimer(ros::Duration(5), &ReconstructionNodelet::timerCb, this);

    recon_frame_id_ = "fixture_1";  // TODO: Use a param

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


void ReconstructionNodelet::axialCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
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

        // TODO: Don't publish the reconstructed cloud at the same rate as the inputs!!!
        //axial_publisher_.publish(axial_point_cloud_);

    } catch (tf2::TransformException& ex) {
        NODELET_INFO_STREAM(node_name_ << ": Could not find transform " << recon_frame_id_ << " to " + msg->header.frame_id + ": " + ex.what());
    }
}


void ReconstructionNodelet::transversalCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
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

        // TODO: Don't publish the reconstructed cloud at the same rate as the inputs!!!
        //transversal_publisher_.publish(trans_point_cloud_);

    } catch (tf2::TransformException& ex) {
        NODELET_INFO_STREAM(node_name_ << ": Could not find transform " << recon_frame_id_ << " to " + msg->header.frame_id + ": " + ex.what());
    }
}


void ReconstructionNodelet::timerCb(const ros::TimerEvent& event){
    NODELET_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");

    axial_publisher_.publish(axial_point_cloud_);
    transversal_publisher_.publish(trans_point_cloud_);
}


} // namespace reconstruction_namespace

PLUGINLIB_EXPORT_CLASS(reconstruction_namespace::ReconstructionNodelet, nodelet::Nodelet);