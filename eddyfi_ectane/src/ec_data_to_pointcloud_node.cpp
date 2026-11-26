#include "ec_data_to_pointcloud_node.h"

#include <rclcpp_components/register_node_macro.hpp>

#include "node_utils.h"



namespace ec_data_to_pointcloud_namespace {

EcDataToPointCloudNode::EcDataToPointCloudNode(const rclcpp::NodeOptions& options)
   : rclcpp::Node("ec_data_to_pointcloud",
                  rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)),
     no_robot_x_pos_(0.0)
{
    no_robot_ = node_utils::requiredParam(*this, "no_robot").as_bool();
    no_robot_extrinsic_frame_id_ = node_utils::requiredParam(*this, "no_robot_extrinsic_frame_id").as_string();
    no_robot_x_pos_increment_ = node_utils::asDouble(node_utils::requiredParam(*this, "no_robot_x_pos_increment"));

    if (no_robot_) {
        br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // One axial and one transversal cloud per frequency
    num_frequencies_ = node_utils::numFrequencies(*this);
    for (int frequency_index = 1; frequency_index <= num_frequencies_; ++frequency_index) {
        const std::string topic_prefix = channel_naming::frequencyNamespace(frequency_index, num_frequencies_);
        axial_publishers_.push_back(create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_prefix + "axial_data_at_probe", node_utils::dataQoS()));
        transversal_publishers_.push_back(create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_prefix + "transversal_data_at_probe", node_utils::dataQoS()));
    }

    // Subscribe last so callbacks only run once everything above is set up
    subscriber_ = create_subscription<eddyfi_ectane::msg::Observation>(
        "eddy_current_data", node_utils::dataQoS(),
        [this](const eddyfi_ectane::msg::Observation::ConstSharedPtr msg) { ectaneCallback(msg); });
}


void EcDataToPointCloudNode::ectaneCallback(const eddyfi_ectane::msg::Observation::ConstSharedPtr msg) {
    // Update dummy TF broadcaster
    if (no_robot_) {
        updateNoRobotTf(*msg);
    }

    // Split the channels by frequency and coil orientation
    std::vector<std::vector<const eddyfi_ectane::msg::Channel*>> axials(num_frequencies_);
    std::vector<std::vector<const eddyfi_ectane::msg::Channel*>> transversals(num_frequencies_);

    for (const auto& channel : msg->channels) {
        const channel_naming::ChannelId id = channel_naming::parseChannelName(channel.name);

        if (id.frequency_index < 1 || id.frequency_index > num_frequencies_) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Dropping channel " << channel.name <<
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
        axial_publishers_[i]->publish(toPointCloud(msg->header, axials[i]));
        transversal_publishers_[i]->publish(toPointCloud(msg->header, transversals[i]));
    }
}


sensor_msgs::msg::PointCloud2 EcDataToPointCloudNode::toPointCloud(
    const std_msgs::msg::Header& header,
    const std::vector<const eddyfi_ectane::msg::Channel*>& channels) const {
    sensor_msgs::msg::PointCloud2 point_cloud;
    point_cloud.header.stamp = header.stamp;
    point_cloud.header.frame_id = header.frame_id;

    sensor_msgs::PointCloud2Modifier modifier(point_cloud);
    modifier.setPointCloud2Fields(
        5,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,          // 32 bits = 4 bytes
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "Imaginary", 1, sensor_msgs::msg::PointField::FLOAT32,
        "Real", 1, sensor_msgs::msg::PointField::FLOAT32
        );

    point_cloud.height = 1;
    point_cloud.width = channels.size();
    point_cloud.is_dense = true;
    point_cloud.point_step = 20;                                 // 5 fields * 4 bytes per field
    point_cloud.row_step = point_cloud.point_step * point_cloud.width;
    point_cloud.data.resize(point_cloud.row_step);

    // Iterators need data, e.g. there is none in the transversal cloud of an axial only configuration
    if (channels.empty()) {
        return point_cloud;
    }

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


void EcDataToPointCloudNode::updateNoRobotTf(const eddyfi_ectane::msg::Observation& msg) {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = msg.header.stamp;
    transformStamped.header.frame_id = no_robot_extrinsic_frame_id_;
    transformStamped.child_frame_id = msg.header.frame_id;
    transformStamped.transform.translation.x = no_robot_x_pos_;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br_->sendTransform(transformStamped);

    no_robot_x_pos_ += no_robot_x_pos_increment_;
}

} // namespace ec_data_to_pointcloud_namespace

RCLCPP_COMPONENTS_REGISTER_NODE(ec_data_to_pointcloud_namespace::EcDataToPointCloudNode)
