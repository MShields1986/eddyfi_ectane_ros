#include "ectane_node.h"

/*
enum Configurations {
    example,
    hf_probe_axial_1_channel,
    hf_probe_transversal_1_channel,
    hf_probe_axial_and_transversal_1_channel,
    hf_probe_axial_and_transversal_4_channels,
    lf_probe_axial_1_channel,
    lf_probe_transversal_1_channel,
    lf_probe_axial_and_transversal_1_channel,
    lf_probe_axial_and_transversal_4_channels
};
*/

EctaneNode::EctaneNode(bool* kill_now)
     : nh_(),
       rate_(10),
       kill_now_(kill_now),
       node_name_(ros::this_node::getName()),
       ns_(ros::this_node::getNamespace()),
       ectane_handler_(
           EctaneNode::paramHandler(ns_ + "/ectane_address", ectane_address_),
           Configurations::lf_probe_axial_and_transversal_4_channels,   // configuration
           EctaneNode::paramHandler(ns_ + "/acquisition_rate", acquisition_rate_),
           EctaneNode::paramHandler(ns_ + "/gain", gain_),
           EctaneNode::paramHandler(ns_ + "/frequency", frequency_),
           EctaneNode::paramHandler(ns_ + "/voltage", voltage_)
       ),
       ec_data_ptr_(ectane_handler_.ec_data_ptr())
{
    // Match the node rate and the Ectane rate
    rate_ = acquisition_rate_;

    initHardware();

    publisher_ = nh_.advertise<eddyfi_ectane::Observation>(ns_ + "/eddy_current_data", 100, true);

    balance_service_ = nh_.advertiseService(ns_ + "/balance_device", &EctaneNode::balanceDevice, this);
    stream_service_ = nh_.advertiseService(ns_ + "/stream_data", &EctaneNode::streamData, this);

    ROS_INFO_STREAM(node_name_ << ": Initialised");
}


// TODO: Consider moving this into the ectane_handler
EctaneNode::~EctaneNode() {
    ROS_INFO_STREAM(node_name_ << ": Shutting down...");
    ectane_handler_.disconnectDevice();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM(node_name_ << ": Shut down");
}


void EctaneNode::initHardware() {
    ROS_INFO_STREAM(node_name_ << ": Initialising Ectane hardware...");
    ectane_handler_.doNothing();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(0.5).sleep();
    ectane_handler_.socketInitialise();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(0.5).sleep();
    ectane_handler_.connecttoDevice();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(0.5).sleep();
    ectane_handler_.setUpDevice();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM(node_name_ << ": Ectane hardware initialised");
}


bool EctaneNode::balanceDevice(eddyfi_ectane::BalanceDevice::Request& request,
                               eddyfi_ectane::BalanceDevice::Response& response) {
    if (request.balance) {
        ROS_INFO_STREAM(node_name_ << ": Balancing device...");
        ectane_handler_.balanceDevice();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
    else {
        response.success = false;
        return true;
    }
}


bool EctaneNode::streamData(eddyfi_ectane::StreamData::Request& request,
                            eddyfi_ectane::StreamData::Response& response) {
    if (request.stream_data) {
        ROS_INFO_STREAM(node_name_ << ": Beginning data stream...");
        ectane_handler_.acquireData();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
    else {
        ROS_INFO_STREAM(node_name_ << ": Stopping data stream...");
        ectane_handler_.stopData();
        ectane_handler_.doNothing();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
}


void EctaneNode::populateEcData() {
    ec_data_.header.frame_id = ns_;
    ec_data_.header.stamp = ros::Time::now();

    // TODO: Channels come in a psuedo-random order and will need sorting
    for (int n_timeslot = 0; n_timeslot < 32; ++n_timeslot) {
        eddyfi_ectane::TimeSlot time_slot = eddyfi_ectane::TimeSlot();
        for (int n_channel = 0; n_channel < 8; ++n_channel) {
            eddyfi_ectane::Channel channel = eddyfi_ectane::Channel();
            // TODO: channel.dc_voltage = ;
            for (int n_freq = 0; n_freq < 8; ++n_freq) {
                eddyfi_ectane::Frequency freq = eddyfi_ectane::Frequency();
                // TODO: freq.imaginary = ;
                // TODO: freq.real = ;
                channel.frequencies[n_freq] = freq;
            }
        time_slot.channels[n_channel] = channel;
        }
    ec_data_.time_slots[n_timeslot] = time_slot;
    }
}


void EctaneNode::run() {
    while (ros::ok() and !*kill_now_) {
        //if (ectane_handler_.state() == State::acquire_data)
        while (ectane_handler_.state() == State::acquire_data) {
            ROS_INFO_STREAM_THROTTLE(10, node_name_ << ": Streaming data...");

            ectane_handler_.readData();
            ROS_INFO_STREAM(node_name_ << ": Data: " << *ec_data_ptr_); // TODO: Drop this in the final version
            
            populateEcData();
            publisher_.publish(ec_data_);

            ros::spinOnce();
            rate_.sleep();
        }

        ROS_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");
        ROS_INFO_STREAM_THROTTLE(10, node_name_ << ": Not streaming data...");
        ros::spinOnce();
        rate_.sleep();
    }
}


template <typename ParamType>
ParamType EctaneNode::paramHandler(std::string param_name, ParamType& param_value)
{
    while (!nh_.hasParam(param_name) and !*kill_now_)
    {
        ROS_INFO_STREAM_THROTTLE(10, node_name_ << ": Waiting for parameter " << param_name);
    }
    nh_.getParam(param_name, param_value);
    ROS_INFO_STREAM(node_name_ << ": Read in parameter " << param_name << " = " << param_value);
    return param_value;
}


// Signal handling
bool KILL_NOW = false;


void killNow(int /*unused*/) {
    KILL_NOW = true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ectane_node");

    signal(SIGTERM, killNow);
    signal(SIGINT, killNow);
    signal(SIGHUP, killNow);

    EctaneNode ectane_node(&KILL_NOW);
    ectane_node.run();

    return 0;
}