#include "reconstruction_node.h"

#include <algorithm>
#include <chrono>

#include <rclcpp_components/register_node_macro.hpp>

#include "node_utils.h"



namespace reconstruction_namespace {

namespace {

// How long after an observation arrives to wait for its transform before dropping it
const rclcpp::Duration kTfTimeout = rclcpp::Duration::from_seconds(1.0);

} // namespace

ReconstructionNode::ReconstructionNode(const rclcpp::NodeOptions& options)
 :  rclcpp::Node("reconstruction",
                 rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)),
    publish_rate_(1)
{
    process_rate_ = node_utils::asDouble(node_utils::requiredParam(*this, "reconstruction.process_rate"));
    publish_rate_ = node_utils::asDouble(node_utils::requiredParam(*this, "reconstruction.publish_rate"));
    recon_frame_id_ = node_utils::requiredParam(*this, "reconstruction.recon_frame_id").as_string();

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_, *this);

    subscriber_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    proc_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pub_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // One axial and one transversal reconstruction per frequency
    const int num_frequencies = node_utils::numFrequencies(*this);
    for (int frequency_index = 1; frequency_index <= num_frequencies; ++frequency_index) {
        const std::string topic_prefix = channel_naming::frequencyNamespace(frequency_index, num_frequencies);
        addStream(topic_prefix, "axial");
        addStream(topic_prefix, "transversal");
    }

    proc_timer_ = create_timer(std::chrono::duration<double>(1.0 / process_rate_),
                               [this]() { timerProcCb(); }, proc_group_);
    pub_timer_ = create_timer(std::chrono::duration<double>(1.0 / publish_rate_),
                              [this]() { timerPubCb(); }, pub_group_);
}


void ReconstructionNode::addStream(const std::string& topic_prefix, const std::string& orientation) {
    streams_.push_back(std::make_unique<ReconstructionStream>());
    ReconstructionStream* stream = streams_.back().get();

    stream->recon_cloud.header.stamp = now();
    stream->recon_cloud.header.frame_id = recon_frame_id_;

    sensor_msgs::PointCloud2Modifier modifier(stream->recon_cloud);
    modifier.setPointCloud2Fields(
        5,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,          // 32 bits = 4 bytes
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "Imaginary", 1, sensor_msgs::msg::PointField::FLOAT32,
        "Real", 1, sensor_msgs::msg::PointField::FLOAT32
        );

    stream->recon_cloud.height = 1;
    stream->recon_cloud.width = 0;
    stream->recon_cloud.is_dense = true;
    stream->recon_cloud.point_step = 20;                         // 5 fields * 4 bytes per field
    stream->recon_cloud.row_step = stream->recon_cloud.point_step * stream->recon_cloud.width;
    stream->recon_cloud.data.resize(stream->recon_cloud.row_step);

    stream->publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_prefix + "distributed_" + orientation + "_reconstruction", node_utils::reconstructionQoS());

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = subscriber_group_;
    stream->subscriber = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_prefix + orientation + "_data_at_probe",
        node_utils::dataQoS(),
        [this, stream](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            BufferedCloud observation{now(), *msg};
            std::lock_guard<std::mutex> lk(stream->buffer_mutex);
            stream->buffer.push_back(std::move(observation));
        },
        subscription_options);
}


void ReconstructionNode::updateReconCloud(ReconstructionStream& stream) {
    // Copy the oldest observation so the buffer is not locked while waiting on TF
    BufferedCloud observation;
    {
        std::lock_guard<std::mutex> lk(stream.buffer_mutex);
        if (stream.buffer.empty()) {
            return;
        }
        observation = stream.buffer.front();
    }
    const sensor_msgs::msg::PointCloud2& msg = observation.cloud;

    // Nothing to add, e.g. the transversal clouds of an axial only configuration, and transforming
    // a cloud without data fails
    if (msg.width == 0 || msg.data.empty()) {
        std::lock_guard<std::mutex> lk(stream.buffer_mutex);
        stream.buffer.pop_front();
        return;
    }

    const rclcpp::Duration waited = now() - observation.received;
    const rclcpp::Duration timeout = waited < kTfTimeout ? kTfTimeout - waited : rclcpp::Duration(0, 0);

    sensor_msgs::msg::PointCloud2 output_pointcloud2;

    try {
        trans_ = tfBuffer_->lookupTransform(recon_frame_id_,                 // target frame
                                            rclcpp::Time(msg.header.stamp),  // target time
                                            msg.header.frame_id,             // source frame
                                            rclcpp::Time(msg.header.stamp),  // source time
                                            recon_frame_id_,                 // fixed frame
                                            timeout                          // time out
                                            );

        tf2::doTransform(msg, output_pointcloud2, trans_);

        {
            std::lock_guard<std::mutex> lk(stream.recon_mutex);
            sensor_msgs::msg::PointCloud2& recon_cloud = stream.recon_cloud;

            recon_cloud.width += output_pointcloud2.width;
            uint64_t prev_size = recon_cloud.data.size();
            recon_cloud.data.resize(recon_cloud.data.size() + output_pointcloud2.data.size());

            std::copy(
                output_pointcloud2.data.begin(),
                output_pointcloud2.data.end(),
                recon_cloud.data.begin() + prev_size);

            recon_cloud.header.stamp = msg.header.stamp;
        }

    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
            "Dropping observation, could not find transform " << recon_frame_id_ <<
            " to " << msg.header.frame_id <<
            ": " << ex.what());
    }

    // Done with the observation either way, retrying one without a transform would stall the stream
    std::lock_guard<std::mutex> lk(stream.buffer_mutex);
    stream.buffer.pop_front();
}


void ReconstructionNode::timerProcCb() {
    for (auto& stream : streams_) {
        updateReconCloud(*stream);
    }
}


void ReconstructionNode::timerPubCb() {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 600000, "Node running");
    for (auto& stream : streams_) {
        std::lock_guard<std::mutex> lk(stream->recon_mutex);
        stream->publisher->publish(stream->recon_cloud);
    }
}


} // namespace reconstruction_namespace

RCLCPP_COMPONENTS_REGISTER_NODE(reconstruction_namespace::ReconstructionNode)
