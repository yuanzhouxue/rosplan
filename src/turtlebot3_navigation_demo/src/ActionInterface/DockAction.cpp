#include "ActionInterface/DockAction.h"

namespace turtlebot3_navigation_demo {

    // constructor
    DockAction::DockAction() : _nh("~") {}


    // action dispatch callback
    bool DockAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        return true;
    }
} // close namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "dock_action");
    turtlebot3_navigation_demo::DockAction rpmb;
    rpmb.runActionInterface();
    return 0;
}
