#include "ec_data_to_pointcloud_node.h"



EcDataToPointCloudNode::EcDataToPointCloudNode(bool* kill_now)
     : nh_(),
       rate_(10),
       kill_now_(kill_now),
       node_name_(ros::this_node::getName()),
       ns_(ros::this_node::getNamespace()),
       subscriber_(nh_.subscribe(ns_ + "/eddy_current_data", 1, &EcDataToPointCloudNode::ectaneCallback, this)),
       axial_publisher_(nh_.advertise<sensor_msgs::PointCloud2>(ns_ + "/axial_data_at_probe", 10, true)),
       transversal_publisher_(nh_.advertise<sensor_msgs::PointCloud2>(ns_ + "/transversal_data_at_probe", 10, true)),
       no_robot_(false),
       no_robot_extrinsic_frame_id_(""),
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
    // TODO: Work on removing repeated code here
    // TODO: Look at what can be instanstiated elsewhere

    // Update dummy TF broadcaster
    if (no_robot_) {
        updateNoRobotTf(msg);
    }

    int num_axials = 0;
    int num_transversals = 0;

    // TODO: Find a faster way to count the number of axials and transversals
    for (const auto& channel : msg->channels) {
        if (channel.name.find("A") != std::string::npos) {
            num_axials++;
        } else if (channel.name.find("T") != std::string::npos) {
            num_transversals++;
        }
    }
    ROS_INFO_STREAM(node_name_ << ": Number of axials: " << num_axials);
    ROS_INFO_STREAM(node_name_ << ": Number of transversals: " << num_transversals);

    // Axial setup
    sensor_msgs::PointCloud2 axial_point_cloud;
    axial_point_cloud.header.stamp = msg->header.stamp;
    axial_point_cloud.header.frame_id = msg->header.frame_id;

    sensor_msgs::PointCloud2Modifier axial_modifier(axial_point_cloud);
    axial_modifier.setPointCloud2Fields(
        5,
        "x", 1, sensor_msgs::PointField::FLOAT32,               // 32 bits = 4 bytes
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "Imaginary", 1, sensor_msgs::PointField::INT32,
        "Real", 1, sensor_msgs::PointField::INT32
        );

    axial_point_cloud.height = 1;
    axial_point_cloud.width = num_axials;
    axial_point_cloud.is_dense = true;
    axial_point_cloud.point_step = 20;                           // 5 fields * 4 bytes per field
    axial_point_cloud.row_step = axial_point_cloud.point_step * axial_point_cloud.width;
    axial_point_cloud.data.resize(axial_point_cloud.row_step);

    // Transversal setup
    sensor_msgs::PointCloud2 trans_point_cloud;
    trans_point_cloud.header.stamp = msg->header.stamp;
    trans_point_cloud.header.frame_id = msg->header.frame_id;

    sensor_msgs::PointCloud2Modifier trans_modifier(trans_point_cloud);
    trans_modifier.setPointCloud2Fields(
        5,
        "x", 1, sensor_msgs::PointField::FLOAT32,               // 32 bits = 4 bytes
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "Imaginary", 1, sensor_msgs::PointField::INT32,
        "Real", 1, sensor_msgs::PointField::INT32
        );

    trans_point_cloud.height = 1;
    trans_point_cloud.width = num_transversals;
    trans_point_cloud.is_dense = true;
    trans_point_cloud.point_step = 20;                           // 5 fields * 4 bytes per field
    trans_point_cloud.row_step = trans_point_cloud.point_step * trans_point_cloud.width;
    trans_point_cloud.data.resize(trans_point_cloud.row_step);

    // Iterators
    sensor_msgs::PointCloud2Iterator<float>      axialIterX(axial_point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float>      axialIterY(axial_point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float>      axialIterZ(axial_point_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float>    axialIterImaginary(axial_point_cloud, "Imaginary");
    sensor_msgs::PointCloud2Iterator<float>    axialIterReal(axial_point_cloud, "Real");

    sensor_msgs::PointCloud2Iterator<float>      transIterX(trans_point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float>      transIterY(trans_point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float>      transIterZ(trans_point_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float>    transIterImaginary(trans_point_cloud, "Imaginary");
    sensor_msgs::PointCloud2Iterator<float>    transIterReal(trans_point_cloud, "Real");

    //int32_t

    for (const auto& channel : msg->channels) {

        if (channel.name.find("A") != std::string::npos) {
            *axialIterX = channel.x_position / 1000;                // mm to m
            *axialIterY = channel.y_position / 1000;                // mm to m
            *axialIterZ = 0.0;
            *axialIterImaginary = channel.imaginary_reading;
            *axialIterReal = channel.real_reading;

            ++axialIterX;
            ++axialIterY;
            ++axialIterZ;
            ++axialIterImaginary;
            ++axialIterReal;
        } else if (channel.name.find("T") != std::string::npos) {
            *transIterX = channel.x_position / 1000;                // mm to m
            *transIterY = channel.y_position / 1000;                // mm to m
            *transIterZ = 0.0;
            *transIterImaginary = channel.imaginary_reading;
            *transIterReal = channel.real_reading;

            ++transIterX;
            ++transIterY;
            ++transIterZ;
            ++transIterImaginary;
            ++transIterReal;
        }
    }
    
    axial_publisher_.publish(axial_point_cloud);
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