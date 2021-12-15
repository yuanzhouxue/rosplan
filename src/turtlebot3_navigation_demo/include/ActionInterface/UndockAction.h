#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rosplan_action_interface/RPActionInterface.h>

namespace turtlebot3_navigation_demo {

    class UndockAction : public KCL_rosplan::RPActionInterface {

    public:
        UndockAction();
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

    private:
        ros::NodeHandle _nh;
        ros::Publisher cmd_vel_pub;
    };
}

