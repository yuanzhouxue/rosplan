#include "ActionInterface/UndockAction.h"

namespace turtlebot3_navigation_demo {

    // constructor
    UndockAction::UndockAction() : _nh("~") {
        std::string vel_topic = _nh.param<std::string>("cmd_vel_topic", "/cmd_vel");
        cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>(vel_topic, 1);
    }

    // action dispatch callback
    bool UndockAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        geometry_msgs::Twist t;
        t.linear.x = -0.1;
        cmd_vel_pub.publish(t);
        ros::Duration(2.0).sleep();
        t.linear.x = 0.0;
        cmd_vel_pub.publish(t);
        return true;
    }
} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "undock_action");

    // create PDDL action subscriber
    turtlebot3_navigation_demo::UndockAction rpmb;

    rpmb.runActionInterface();

    return 0;
}
