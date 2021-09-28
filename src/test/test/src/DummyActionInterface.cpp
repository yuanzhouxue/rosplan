#include "DummyActionInterface.h"

bool rosplane::DummyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
    ros::NodeHandle nh("~");
    // set action name
    std::string action_name;
    nh.getParam("pddl_action_name", action_name);
    ROS_INFO("(%s): concreteCallback", action_name.c_str());
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dummyaction_node");
    rosplane::DummyActionInterface dsi;

    dsi.runActionInterface();
    return 0;
}