#include "ectane_node.h"


EctaneNode::EctaneNode(bool* kill_now)
     : nh_(),
       rate_(1),
       kill_now_(kill_now),
       node_name_(ros::this_node::getName()),
       ns_(ros::this_node::getNamespace()),
       ectane_handler_()
{
    ROS_INFO_STREAM(node_name_ << ": Initialised");
}


void EctaneNode::run()
{
    while (ros::ok() and !*kill_now_)
    {
        ectane_handler_.run();

        ROS_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");
        ros::spinOnce();
        rate_.sleep();
    }
}


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