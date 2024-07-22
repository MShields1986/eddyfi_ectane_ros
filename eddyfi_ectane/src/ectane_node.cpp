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
       rate_(20),
       kill_now_(kill_now),
       node_name_(ros::this_node::getName()),
       ns_(ros::this_node::getNamespace()),
       ectane_handler_(
       // TODO: Add configs here
        )
{
    initHardware();

    publisher_ = nh_.advertise<geometry_msgs::Twist>(ns_ + "/eddy_current_data", 100, true);

    balance_service_ = nh_.advertiseService(ns_ + "/balance_device", &EctaneNode::balanceDevice, this);
    stream_service_ = nh_.advertiseService(ns_ + "/stream_data", &EctaneNode::streamData, this);

    ROS_INFO_STREAM(node_name_ << ": Initialised");
}


EctaneNode::~EctaneNode()
{
    ROS_INFO_STREAM(node_name_ << ": Shutting down...");
    ectane_handler_.disconnectDevice();
    // TODO: Check to see if the C SDK handshakes to confirm state receipt
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM(node_name_ << ": Shut down");
}


void EctaneNode::initHardware()
{
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
                               eddyfi_ectane::BalanceDevice::Response& response)
{
    if (request.balance)
    {
        ROS_INFO_STREAM(node_name_ << ": Balancing device...");
        ectane_handler_.balanceDevice();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
    else
    {
        response.success = false;
        return true;
    }
}


bool EctaneNode::streamData(eddyfi_ectane::StreamData::Request& request,
                            eddyfi_ectane::StreamData::Response& response)
{
    if (request.stream_data)
    {
        ROS_INFO_STREAM(node_name_ << ": Beginning data stream...");
        ectane_handler_.acquireData();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
    else
    {
        ROS_INFO_STREAM(node_name_ << ": Stopping data stream...");
        ectane_handler_.stopData();
        ectane_handler_.doNothing();
        // TODO: Check to see if the C SDK handshakes to confirm state receipt
        response.success = true;
        return true;
    }
}


void EctaneNode::run()
{
    while (ros::ok() and !*kill_now_)
    {
        if (ectane_handler_.state() == State::acquire_data)
        {
            ROS_INFO_STREAM_THROTTLE(10, node_name_ << ": Streaming data...");
            geometry_msgs::Twist msg;
            publisher_.publish(msg);
        }

        ROS_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");
        ros::spinOnce();
        rate_.sleep();
    }
}


/*
template <typename ParamType>
void EctaneNode::paramHandler(std::string param_name, ParamType& param_value)
{
    while (!nh_.hasParam(param_name) and !*kill_now_)
    {
        ROS_INFO_STREAM_THROTTLE(10, node_name_ << ": Waiting for parameter " << param_name);
    }
    nh_.getParam(param_name, param_value);
    ROS_INFO_STREAM(node_name_ << ": Read in parameter " << param_name << " = " << param_value);
}
*/


// Signal handling
bool KILL_NOW = false;


void killNow(int /*unused*/) 
{
    KILL_NOW = true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ectane_node");

    signal(SIGTERM, killNow);
    signal(SIGINT, killNow);
    signal(SIGHUP, killNow);

    EctaneNode ectane_node(&KILL_NOW);
    ectane_node.run();

    return 0;
}