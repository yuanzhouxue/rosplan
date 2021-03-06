#include "goto_waypoint.h"

namespace ros_app {

    // constructor
    goto_waypoint_ActionInterface::goto_waypoint_ActionInterface(std::string &actionserver) : _nh("~"), action_client_(actionserver, true) {
        // get waypoints reference frame from param server
        _nh.param<std::string>("waypoint_frameid", waypoint_frameid_, "map");
        _nh.param<std::string>("wp_namespace", wp_namespace_, "/rosplan_demo_waypoints/wp");
        head_cmd = _nh.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1);

        // setup a move base clear costmap client (to be able to send clear costmap requests later on)
        clear_costmaps_client_ = _nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    }

    bool goto_waypoint_ActionInterface::wpIDtoPoseStamped(std::string wpID, geometry_msgs::PoseStamped &result) {
        std::vector<double> wp;
        if(_nh.hasParam(wp_namespace_ + "/" + wpID)) {
            if(_nh.getParam(wp_namespace_ + "/" + wpID, wp)) {
                if(wp.size() == 3) {
                    result.header.frame_id = waypoint_frameid_;
                    result.pose.position.x = wp[0];
                    result.pose.position.y = wp[1];

                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, wp[2]);
                    result.pose.orientation.x = q[0];
                    result.pose.orientation.y = q[1];
                    result.pose.orientation.z = q[2];
                    result.pose.orientation.w = q[3];

                    return true;
                }
                else {
                    ROS_ERROR("wp size must be equal to 3 : (x, y, and theta)");
                    return false;
                }
            }
        }
        else
            return false;
    }

    // action dispatch callback
    bool goto_waypoint_ActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        trajectory_msgs::JointTrajectory jt;
        jt.joint_names = { "head_1_joint", "head_2_joint" };
        trajectory_msgs::JointTrajectoryPoint jtp;
        jtp.positions = { 0.0, -0.75 };
        jtp.time_from_start = ros::Duration(2.0);
        jt.points.push_back(jtp);
        head_cmd.publish(jt);

        // get waypoint ID from action dispatch msg
        std::string wpID;
        bool found = false;
        // iterating over parameters (e.g. kenny, wp0, wp1)
        for(size_t i = 0; i < msg->parameters.size(); i++) {
            // check their keys
            if(0 == msg->parameters[i].key.compare("to") or 0 == msg->parameters[i].key.compare("w1")) {
                // wp id found in msg params
                wpID = msg->parameters[i].value;
                found = true;
            }
        }
        if(!found) {
            ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?to", params.name.c_str());
            return false;
        }

        // get waypoint coordinates from its ID via query to parameter server
        geometry_msgs::PoseStamped pose;
        if(!wpIDtoPoseStamped(wpID, pose)) {
            ROS_ERROR("Waypoint not found in parameter server");
            return false;
        }

        ROS_INFO("KCL: (%s) waiting for move_base action server to start", params.name.c_str());
        action_client_.waitForServer();

        // dispatch MoveBase action
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = pose;
        action_client_.sendGoal(goal);

        ROS_INFO("KCL: (%s) waiting fot move_base to finish", params.name.c_str());
        bool finished_before_timeout = action_client_.waitForResult();
        if (finished_before_timeout) {

            actionlib::SimpleClientGoalState state = action_client_.getState();
            ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());

            if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {

                // publish feedback (achieved)
                return true;

            } else {

                // clear costmaps
                std_srvs::Empty emptySrv;
                clear_costmaps_client_.call(emptySrv);

                // publish feedback (failed)
                return false;
            }
        } else {
            // timed out (failed)
            action_client_.cancelAllGoals();
            ROS_INFO("KCL: (%s) action timed out", params.name.c_str());
            return false;
        }
    }
} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_movebase");

    ros::NodeHandle nh("~");
    std::string actionserver;
    nh.param("action_server", actionserver, std::string("/move_base"));

    // create PDDL action subscriber
    ros_app::goto_waypoint_ActionInterface rpmb(actionserver);

    rpmb.runActionInterface();

    return 0;
}
