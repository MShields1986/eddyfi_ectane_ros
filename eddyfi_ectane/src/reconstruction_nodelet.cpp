#include "reconstruction_nodelet.h"



namespace reconstruction_namespace {

namespace {

// How long after an observation arrives to wait for its transform before dropping it
const ros::Duration kTfTimeout(1.0);

} // namespace

ReconstructionNodelet::ReconstructionNodelet()
 :  publish_rate_(1),
    ns_("/ectane"),
    tfListener_(tfBuffer_)
{
}


void ReconstructionNodelet::onInit()
{
    ros::NodeHandle &nh_ = getMTNodeHandle(); //getNodeHandle();
    node_name_ = getName();
    ns_ = getMTNodeHandle().getNamespace();

    ReconstructionNodelet::paramHandler(ns_ + "/settings/reconstruction/process_rate", process_rate_);
    ReconstructionNodelet::paramHandler(ns_ + "/settings/reconstruction/publish_rate", publish_rate_);
    ReconstructionNodelet::paramHandler(ns_ + "/settings/reconstruction/recon_frame_id", recon_frame_id_);

    // One axial and one transversal reconstruction per frequency
    const int num_frequencies = channel_naming::waitForNumFrequencies(ns_ + "/crosstalk/");
    for (int frequency_index = 1; frequency_index <= num_frequencies; ++frequency_index) {
        const std::string topic_prefix =
            ns_ + "/" + channel_naming::frequencyNamespace(frequency_index, num_frequencies);
        addStream(nh_, topic_prefix, "axial");
        addStream(nh_, topic_prefix, "transversal");
    }

    proc_timer_ = nh_.createTimer(ros::Duration(1.0f / process_rate_), &ReconstructionNodelet::timerProcCb, this);
    pub_timer_ = nh_.createTimer(ros::Duration(1.0f / publish_rate_), &ReconstructionNodelet::timerPubCb, this);
}


template <typename ParamType>
ParamType ReconstructionNodelet::paramHandler(std::string param_name, ParamType& param_value) {
    while (!nh_.hasParam(param_name)) {
        NODELET_INFO_STREAM_THROTTLE(10, node_name_ << ": Waiting for parameter " << param_name);
        ros::Duration(0.1).sleep();
    }
    nh_.getParam(param_name, param_value);
    NODELET_DEBUG_STREAM(node_name_ <<
        ": Read in parameter " << param_name << " = " << param_value);
    return param_value;
}


void ReconstructionNodelet::addStream(ros::NodeHandle& nh,
                                      const std::string& topic_prefix,
                                      const std::string& orientation) {
    streams_.push_back(std::make_unique<ReconstructionStream>());
    ReconstructionStream* stream = streams_.back().get();

    stream->recon_cloud.header.stamp = ros::Time::now();
    stream->recon_cloud.header.frame_id = recon_frame_id_;

    sensor_msgs::PointCloud2Modifier modifier(stream->recon_cloud);
    modifier.setPointCloud2Fields(
        5,
        "x", 1, sensor_msgs::PointField::FLOAT32,               // 32 bits = 4 bytes
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "Imaginary", 1, sensor_msgs::PointField::FLOAT32,
        "Real", 1, sensor_msgs::PointField::FLOAT32
        );

    stream->recon_cloud.height = 1;
    stream->recon_cloud.width = 0;
    stream->recon_cloud.is_dense = true;
    stream->recon_cloud.point_step = 20;                         // 5 fields * 4 bytes per field
    stream->recon_cloud.row_step = stream->recon_cloud.point_step * stream->recon_cloud.width;
    stream->recon_cloud.data.resize(stream->recon_cloud.row_step);

    stream->publisher = nh.advertise<sensor_msgs::PointCloud2>(
        topic_prefix + "distributed_" + orientation + "_reconstruction", 100, true);

    stream->subscriber = nh.subscribe<sensor_msgs::PointCloud2>(
        topic_prefix + orientation + "_data_at_probe",
        6000,
        [stream](const sensor_msgs::PointCloud2::ConstPtr& msg) {
            std::lock_guard<std::mutex> lk(stream->buffer_mutex);
            stream->buffer.push_back({ros::Time::now(), *msg});
        });
}


void ReconstructionNodelet::updateReconCloud(ReconstructionStream& stream) {
    // Copy the oldest observation so the buffer is not locked while waiting on TF
    BufferedCloud observation;
    {
        std::lock_guard<std::mutex> lk(stream.buffer_mutex);
        if (stream.buffer.empty()) {
            return;
        }
        observation = stream.buffer.front();
    }
    const sensor_msgs::PointCloud2& msg = observation.cloud;

    const ros::Duration waited = ros::Time::now() - observation.received;
    const ros::Duration timeout = waited < kTfTimeout ? kTfTimeout - waited : ros::Duration(0);

    sensor_msgs::PointCloud2 output_pointcloud2;

    try {
        trans_ = tfBuffer_.lookupTransform(recon_frame_id_,      // target frame
                                           msg.header.stamp,     // target time
                                           msg.header.frame_id,  // source frame
                                           msg.header.stamp,     // source time
                                           recon_frame_id_,      // fixed frame
                                           timeout               // time out
                                           );

        tf2::doTransform<sensor_msgs::PointCloud2>(msg, output_pointcloud2, trans_);

        {
            std::lock_guard<std::mutex> lk(stream.recon_mutex);
            sensor_msgs::PointCloud2& recon_cloud = stream.recon_cloud;

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
        NODELET_WARN_STREAM_THROTTLE(1, node_name_ <<
            ": Dropping observation, could not find transform " << recon_frame_id_ <<
            " to " << msg.header.frame_id <<
            ": " << ex.what());
    }

    // Done with the observation either way, retrying one without a transform would stall the stream
    std::lock_guard<std::mutex> lk(stream.buffer_mutex);
    stream.buffer.pop_front();
}


void ReconstructionNodelet::timerProcCb(const ros::TimerEvent& event){
    for (auto& stream : streams_) {
        updateReconCloud(*stream);
    }
}


void ReconstructionNodelet::timerPubCb(const ros::TimerEvent& event){
    NODELET_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");
    for (auto& stream : streams_) {
        std::lock_guard<std::mutex> lk(stream->recon_mutex);
        stream->publisher.publish(stream->recon_cloud);
    }
}


} // namespace reconstruction_namespace

PLUGINLIB_EXPORT_CLASS(reconstruction_namespace::ReconstructionNodelet, nodelet::Nodelet);
