#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rosplan_action_interface/RPActionInterface.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace ros_app {

    class goto_waypoint_ActionInterface : public KCL_rosplan::RPActionInterface {

    private:
        ros::NodeHandle _nh;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
        ros::ServiceClient clear_costmaps_client_;
        std::string waypoint_frameid_;
        std::string wp_namespace_;
        ros::Publisher head_cmd;

    public:
        goto_waypoint_ActionInterface(std::string& actionserver);
        bool wpIDtoPoseStamped(std::string wpID, geometry_msgs::PoseStamped& result);
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

    };
}
