#include "ec_data_to_pointcloud_nodelet.h"



namespace ec_data_to_pointcloud_namespace {

EcDataToPointCloudNodelet::EcDataToPointCloudNodelet()
   : ns_("/ectane")
{
}


void EcDataToPointCloudNodelet::onInit()
{
    ros::NodeHandle &nh_ = getMTNodeHandle(); //getNodeHandle();

    node_name_ = getName();
    ns_ = getMTNodeHandle().getNamespace();

    no_robot_ = false;
    no_robot_extrinsic_frame_id_ = "";
    no_robot_x_pos_increment_ = 0.0;
    no_robot_x_pos_ = 0.0;

    EcDataToPointCloudNodelet::paramHandler(ns_ + "/settings/no_robot", no_robot_);
    EcDataToPointCloudNodelet::paramHandler(ns_ + "/settings/no_robot_extrinsic_frame_id", no_robot_extrinsic_frame_id_);
    EcDataToPointCloudNodelet::paramHandler(ns_ + "/settings/no_robot_x_pos_increment", no_robot_x_pos_increment_);

    // Work around to allow concurrent callbacks on the same topic
    // By default you there is no concurrency on callbacks for each topic!
    //ros::SubscribeOptions ops;
    //ops.template init<std_msgs::String>(ns_ + "/eddy_current_data", 100, &EcDataToPointCloudNodelet::ectaneCallback, this);
    //ops.allow_concurrent_callbacks = true;
    //subscriber_ = nh_.subscribe(ops);

    // One axial and one transversal cloud per frequency
    num_frequencies_ = channel_naming::waitForNumFrequencies(ns_ + "/crosstalk/");
    for (int frequency_index = 1; frequency_index <= num_frequencies_; ++frequency_index) {
        const std::string topic_prefix =
            ns_ + "/" + channel_naming::frequencyNamespace(frequency_index, num_frequencies_);
        axial_publishers_.push_back(
            nh_.advertise<sensor_msgs::PointCloud2>(topic_prefix + "axial_data_at_probe", 10000, true));
        transversal_publishers_.push_back(
            nh_.advertise<sensor_msgs::PointCloud2>(topic_prefix + "transversal_data_at_probe", 10000, true));
    }

    // Subscribe last so callbacks only run once everything above is set up
    subscriber_ = nh_.subscribe(ns_ + "/eddy_current_data", 10000, &EcDataToPointCloudNodelet::ectaneCallback, this);
}


template <typename ParamType>
void EcDataToPointCloudNodelet::paramHandler(std::string param_name, ParamType& param_value) {
    while (!nh_.hasParam(param_name)) {
        NODELET_INFO_STREAM_THROTTLE(10, node_name_ << ": Waiting for parameter " << param_name);
        ros::Duration(0.1).sleep();
    }
    nh_.getParam(param_name, param_value);
    NODELET_DEBUG_STREAM(node_name_ << ": Read in parameter " << param_name << " = " << param_value);
    //return param_value;
}


void EcDataToPointCloudNodelet::ectaneCallback(const eddyfi_ectane::Observation::ConstPtr& msg) {
    // Update dummy TF broadcaster
    if (no_robot_) {
        updateNoRobotTf(msg);
    }

    // Split the channels by frequency and coil orientation
    std::vector<std::vector<const eddyfi_ectane::Channel*>> axials(num_frequencies_);
    std::vector<std::vector<const eddyfi_ectane::Channel*>> transversals(num_frequencies_);

    for (const auto& channel : msg->channels) {
        const channel_naming::ChannelId id = channel_naming::parseChannelName(channel.name);

        if (id.frequency_index < 1 || id.frequency_index > num_frequencies_) {
            NODELET_WARN_STREAM_THROTTLE(10, node_name_ << ": Dropping channel " << channel.name <<
                " as its frequency is not in the crosstalk configuration");
            continue;
        }

        if (id.coil.find("A") != std::string::npos) {
            axials[id.frequency_index - 1].push_back(&channel);
        } else if (id.coil.find("T") != std::string::npos) {
            transversals[id.frequency_index - 1].push_back(&channel);
        }
    }

    for (int i = 0; i < num_frequencies_; ++i) {
        axial_publishers_[i].publish(toPointCloud(msg->header, axials[i]));
        transversal_publishers_[i].publish(toPointCloud(msg->header, transversals[i]));
    }
}


sensor_msgs::PointCloud2 EcDataToPointCloudNodelet::toPointCloud(
    const std_msgs::Header& header,
    const std::vector<const eddyfi_ectane::Channel*>& channels) const {
    sensor_msgs::PointCloud2 point_cloud;
    point_cloud.header.stamp = header.stamp;
    point_cloud.header.frame_id = header.frame_id;

    sensor_msgs::PointCloud2Modifier modifier(point_cloud);
    modifier.setPointCloud2Fields(
        5,
        "x", 1, sensor_msgs::PointField::FLOAT32,               // 32 bits = 4 bytes
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "Imaginary", 1, sensor_msgs::PointField::FLOAT32,
        "Real", 1, sensor_msgs::PointField::FLOAT32
        );

    point_cloud.height = 1;
    point_cloud.width = channels.size();
    point_cloud.is_dense = true;
    point_cloud.point_step = 20;                                 // 5 fields * 4 bytes per field
    point_cloud.row_step = point_cloud.point_step * point_cloud.width;
    point_cloud.data.resize(point_cloud.row_step);

    sensor_msgs::PointCloud2Iterator<float>      iterX(point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float>      iterY(point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float>      iterZ(point_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float>      iterImaginary(point_cloud, "Imaginary");
    sensor_msgs::PointCloud2Iterator<float>      iterReal(point_cloud, "Real");

    for (const auto* channel : channels) {
        *iterX = channel->x_position / 1000;                     // mm to m
        *iterY = channel->y_position / 1000;                     // mm to m
        *iterZ = 0.0;
        *iterImaginary = channel->imaginary_reading;
        *iterReal = channel->real_reading;

        ++iterX;
        ++iterY;
        ++iterZ;
        ++iterImaginary;
        ++iterReal;
    }

    return point_cloud;
}


void EcDataToPointCloudNodelet::updateNoRobotTf(const eddyfi_ectane::Observation::ConstPtr& msg) {
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

    no_robot_x_pos_ += no_robot_x_pos_increment_;
}

} // namespace ec_data_to_pointcloud_namespace

PLUGINLIB_EXPORT_CLASS(ec_data_to_pointcloud_namespace::EcDataToPointCloudNodelet, nodelet::Nodelet);