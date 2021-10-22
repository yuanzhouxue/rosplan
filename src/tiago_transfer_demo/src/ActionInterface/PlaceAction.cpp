#include "ActionInterface/PlaceAction.h"
#include <moveit/move_group_interface/move_group_interface.h>

#define INFO(...) ROS_INFO(__VA_ARGS__)

using namespace std;
using rosplan_knowledge_msgs::GetInstanceService;
using rosplan_knowledge_msgs::KnowledgeItem;
using rosplan_knowledge_msgs::KnowledgeUpdateService;
using rosplan_knowledge_msgs::KnowledgeUpdateServiceArray;

namespace rosplane {

    /* constructor */
    PlaceAction::PlaceAction() : nh_("~"), play_m_as("/play_motion"), place_as("/place_pose"), tf_l(tfBuffer) {
        node_name = ros::this_node::getName();
        clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        update_knowledge_client_ = nh_.serviceClient<KnowledgeUpdateService>("/rosplan_knowledge_base/update");
        update_knowledge_array_client_ = nh_.serviceClient<KnowledgeUpdateServiceArray>("/rosplan_knowledge_base/update_array");
        query_knowledge_client_ = nh_.serviceClient<GetInstanceService>("/rosplan_knowledge_base/state/instances");
        pub_arm_topic = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);
        ROS_INFO("(%s): Initalizing...", node_name.c_str());
        // auto tfBuffer = tf2_ros::Buffer();
        // tf2_ros::TransformListener tf_l(tfBuffer);
        ROS_INFO("(%s): Waiting for /place_pose AS...", node_name.c_str());
        if (!place_as.waitForServer(ros::Duration(20))) {
            ROS_ERROR("(%s): Could not connect to /place_pose AS", node_name.c_str());
            exit(0);
        }

        ROS_INFO("(%s): Setting publishers to torso and head controller...", node_name.c_str());
        torso_cmd = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
        head_cmd = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1);
        gripper_cmd = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1);
        detected_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("/detected_aruco_pose", 1, true);

        ROS_INFO("(%s): Waiting for '/play_motion' AS...", node_name.c_str());
        if (!play_m_as.waitForServer(ros::Duration(20))) {
            ROS_ERROR("(%s): Could not connect to /play_motion AS", node_name.c_str());
            exit(0);
        }

        ROS_INFO("(%s): Connected!", node_name.c_str());
        ROS_INFO("(%s): Ready to receive.", node_name.c_str());

        service = nh_.advertiseService("my_place_service", &PlaceAction::callback, this);
    }

    bool PlaceAction::callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        const rosplan_dispatch_msgs::ActionDispatch a;
        rosplan_dispatch_msgs::ActionDispatch::ConstPtr pa(new rosplan_dispatch_msgs::ActionDispatch());
        return concreteCallback(pa);
    }

    string PlaceAction::stripLeadingSlash(const string& s) {
        if (s[0] == '/') return std::move(s.substr(1));
        return std::move(s.substr());
    }

    void PlaceAction::prepareRobot() {
        ROS_INFO("(%s): Unfold arm safely", node_name.c_str());
        play_motion_msgs::PlayMotionGoal pmg;
        pmg.motion_name = "pregrasp";
        pmg.skip_planning = false;
        play_m_as.sendGoalAndWait(pmg);
        ROS_INFO("(%s): Done.", node_name.c_str());
        lowerHead();
        ROS_INFO("(%s): Robot prepared.", node_name.c_str());
    }

    void PlaceAction::lowerHead() {
        ROS_INFO("(%s): Moving head down...", node_name.c_str());
        trajectory_msgs::JointTrajectory jt;
        jt.joint_names = { "head_1_joint", "head_2_joint" };
        trajectory_msgs::JointTrajectoryPoint jtp;
        jtp.positions = { 0.0, -0.75 };
        jtp.time_from_start = ros::Duration(2.0);
        jt.points.push_back(jtp);
        head_cmd.publish(jt);
        ROS_INFO("(%s): Lower head done.", node_name.c_str());
        ROS_INFO("(%s): Looking for aruco...", node_name.c_str());
        ros::ServiceClient look_around_client = nh_.serviceClient<std_srvs::Empty>("/look_around_interface/tiago_look_around");
        std_srvs::Empty e;
        look_around_client.call(e);
        ROS_INFO("(%s): Done.", node_name.c_str());
    }

    void PlaceAction::liftTorso() {
        ROS_INFO("(%s): Moving torso up...", node_name.c_str());
        trajectory_msgs::JointTrajectory jt;
        jt.joint_names = { "torso_lift_joint" };
        trajectory_msgs::JointTrajectoryPoint jtp;
        jtp.positions = { 0.34 };
        jtp.time_from_start = ros::Duration(2.0);
        jt.points.push_back(jtp);
        head_cmd.publish(jt);
        ROS_INFO("(%s): Lift torso done.", node_name.c_str());
    }


    /* action dispatch callback */
    bool PlaceAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        liftTorso();
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "base_footprint";
        goal_pose.pose.position.x = 0.5f;
        goal_pose.pose.position.y = 0.0f;
        goal_pose.pose.position.z = 1.0f;
        goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57f, 0.f, 0.f);
        std::vector<std::string> torso_arm_joint_names;
        //select group of joints
        moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
        //choose your preferred planner
        group_arm_torso.setPlannerId("SBLkConfigDefault");
        group_arm_torso.setPoseReferenceFrame("base_footprint");
        group_arm_torso.setPoseTarget(goal_pose);

        ROS_INFO_STREAM("Planning to move " <<
            group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
            group_arm_torso.getPlanningFrame());

        group_arm_torso.setStartStateToCurrentState();
        group_arm_torso.setMaxVelocityScalingFactor(1.0);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        //set maximum time to find a plan
        group_arm_torso.setPlanningTime(5.0);
        bool success = bool(group_arm_torso.plan(my_plan));

        if (!success)
            throw std::runtime_error("No plan found");

        ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

        ROS_INFO("(%s): Gonna place the object on the table.", ros::this_node::getName().c_str());
        // Execute the plan
        ros::Time start = ros::Time::now();
        moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
        if (!bool(e))
            throw std::runtime_error("Error executing plan");
        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
        ROS_INFO("(%s): Done.", ros::this_node::getName().c_str());
        trajectory_msgs::JointTrajectory jt;
        jt.joint_names = { "gripper_left_finger_joint", "gripper_right_finger_joint" };
        trajectory_msgs::JointTrajectoryPoint jtp;
        jtp.positions = { 0.044, 0.044 };
        jtp.time_from_start = ros::Duration(1.0);
        jt.points.push_back(jtp);
        gripper_cmd.publish(jt);
        ros::Duration(2.0).sleep();
        return true;
    }

    // rosplan_knowledge_msgs::KnowledgeItem createKnowledgeItem(uint8_t k_type, const string& attribute_name, )
} // close namespace

    /*-------------*/
    /* Main method */
    /*-------------*/

int main(int argc, char** argv) {

    ros::init(argc, argv, "place_action_interface");
    // ros::NodeHandle nh("~");
    // std::string actionserver;
    // nh.param("action_server", actionserver, std::string("/move_base"));
    // create PDDL action subscriber
    rosplane::PlaceAction rpti;

    rpti.runActionInterface();

    return 0;
}