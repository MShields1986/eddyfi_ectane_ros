#include "reconstruction_nodelet.h"



namespace reconstruction_namespace {

ReconstructionNodelet::ReconstructionNodelet()
 :  publish_rate_(1),
    ns_("/ectane"), // TODO: Figure out how to get this when using nodelets so it isn't hardcoded
    tfListener_(tfBuffer_)
{
}


void ReconstructionNodelet::onInit()
{
    ros::NodeHandle &nh_ = getMTNodeHandle(); //getNodeHandle();
    node_name_ = getName();
    //ns_ = ros::this_node::getNamespace();

    axial_subscriber_ = nh_.subscribe(ns_ + "/axial_data_at_probe", 6000, &ReconstructionNodelet::axialCallback, this);
    transversal_subscriber_ = nh_.subscribe(ns_ + "/transversal_data_at_probe", 6000, &ReconstructionNodelet::transversalCallback, this);

    axial_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(ns_ + "/distributed_axial_reconstruction", 100, true);
    transversal_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(ns_ + "/distributed_transversal_reconstruction", 100, true);

    ReconstructionNodelet::paramHandler(ns_ + "/reconstruction/rate", publish_rate_);
    timer_ = nh_.createTimer(ros::Duration(1 / publish_rate_), &ReconstructionNodelet::timerCb, this);

    ReconstructionNodelet::paramHandler(ns_ + "/reconstruction/recon_frame_id", recon_frame_id_);

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


template <typename ParamType>
ParamType ReconstructionNodelet::paramHandler(std::string param_name, ParamType& param_value) {
    while (!nh_.hasParam(param_name)) {
        NODELET_INFO_STREAM_THROTTLE(10, node_name_ << ": Waiting for parameter " << param_name);
    }
    nh_.getParam(param_name, param_value);
    NODELET_DEBUG_STREAM(node_name_ << 
        ": Read in parameter " << param_name << " = " << param_value);
    return param_value;
}


void ReconstructionNodelet::axialCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    axial_buffer_.push_back(*msg);
}


void ReconstructionNodelet::transversalCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    trans_buffer_.push_back(*msg);
}


void ReconstructionNodelet::updateReconCloud(std::deque<sensor_msgs::PointCloud2>& buffer, sensor_msgs::PointCloud2& recon_cloud) {
    if (!buffer.empty()) {
        sensor_msgs::PointCloud2* msg = &buffer.front();
        sensor_msgs::PointCloud2 output_pointcloud2;

        try {
            trans_ = tfBuffer_.lookupTransform(recon_frame_id_,      // target frame
                                               msg->header.stamp,    // target time
                                               msg->header.frame_id, // source frame
                                               msg->header.stamp,    // source time
                                               // "map",                // fixed frame
                                               recon_frame_id_,      // fixed frame
                                               ros::Duration(3.0)    // time out
                                               );

            tf2::doTransform<sensor_msgs::PointCloud2>(*msg, output_pointcloud2, trans_);

            recon_cloud.width += output_pointcloud2.width;
            uint64_t prev_size = recon_cloud.data.size();
            recon_cloud.data.resize(recon_cloud.data.size() + output_pointcloud2.data.size());

            std::copy(
                output_pointcloud2.data.begin(),
                output_pointcloud2.data.end(),
                recon_cloud.data.begin() + prev_size);

            recon_cloud.header.stamp = msg->header.stamp;

            buffer.pop_front();

        } catch (tf2::TransformException& ex) {
            NODELET_WARN_STREAM(node_name_ << 
                ": Could not find transform " << recon_frame_id_ << 
                " to " << msg->header.frame_id << 
                ": " << ex.what());
        }
    }
}


void ReconstructionNodelet::timerCb(const ros::TimerEvent& event){
    NODELET_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");

    // TODO: This might be better as two separate timers
    updateReconCloud(axial_buffer_, axial_point_cloud_);
    axial_publisher_.publish(axial_point_cloud_);
    updateReconCloud(trans_buffer_, trans_point_cloud_);
    transversal_publisher_.publish(trans_point_cloud_);
}


} // namespace reconstruction_namespace

PLUGINLIB_EXPORT_CLASS(reconstruction_namespace::ReconstructionNodelet, nodelet::Nodelet);