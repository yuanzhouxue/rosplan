#ifndef PICK_ACTION_H
#define PICK_ACTION_H


#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <cstdlib>

#include "rosplan_action_interface/RPActionInterface.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include <diagnostic_msgs/KeyValue.h>
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include <actionlib/action_definition.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_pick_demo/PickUpPoseAction.h>
#include <tiago_pick_demo/PickUpPoseGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <play_motion_msgs/PlayMotionAction.h>
#include <play_motion_msgs/PlayMotionGoal.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <control_msgs/JointTrajectoryControllerState.h>


using std::string;
using std::vector;

/**
 * This file defines an action interface created in tutorial 10.
 */
namespace rosplane {

    class PickUpAction : public KCL_rosplan::RPActionInterface {

    private:
        // action lib client that will make the communication with move_base action server
        actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> play_m_as;
        actionlib::SimpleActionClient<tiago_pick_demo::PickUpPoseAction> pick_as;

        ros::Publisher torso_cmd, head_cmd, detected_pos_pub;

        // to clear costmaps if move base gets stuck
        ros::ServiceClient clear_costmaps_client_;
        ros::ServiceClient update_knowledge_client_;
        ros::ServiceClient update_knowledge_array_client_;
        ros::ServiceClient query_knowledge_client_;
        ros::ServiceServer service;


        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tf_l;
        // tf::TransformListener tf_1;

        /// nodehandle, manages e.g. subscriptions and publications
        ros::NodeHandle nh_;

        bool robot_prepared = false;

        /// waypoints reference frame
        std::string waypoint_frameid_;
        std::string wp_namespace_;
        std::string node_name;
        double probility;
        string stripLeadingSlash(const string& s);
        void prepareRobot();
        void lowerHead();
        void liftTorso();
        bool callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    public:

        /* constructor */
        PickUpAction();

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}
#endif