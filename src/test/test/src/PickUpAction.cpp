#include "PickUpAction.h"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
using rosplan_knowledge_msgs::GetInstanceService;
using rosplan_knowledge_msgs::KnowledgeItem;
using rosplan_knowledge_msgs::KnowledgeUpdateService;
using rosplan_knowledge_msgs::KnowledgeUpdateServiceArray;

/* The implementation of RPTutorial.h */
namespace rosplane {

    /* constructor */
    PickUpAction::PickUpAction() : nh_("~"), play_m_as("/play_motion"), pick_as("/pickup_pose"), tf_l(tfBuffer) {
        node_name = ros::this_node::getName();
        // create a node handle to manage communication with ROS network
        // ros::NodeHandle nh("~");
        // get waypoints reference frame from param server
        nh_.param<std::string>("waypoint_frameid", waypoint_frameid_, "map");
        nh_.param<std::string>("wp_namespace", wp_namespace_, "/rosplan_demo_waypoints/wp");
        nh_.param<double>("probility", probility, 1.0);
        // ROS_INFO("Simulated movebase with probility %lf", probility);
        // setup a move base clear costmap client (to be able to send clear costmap requests later on)
        clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        update_knowledge_client_ = nh_.serviceClient<KnowledgeUpdateService>("/rosplan_knowledge_base/update");
        update_knowledge_array_client_ = nh_.serviceClient<KnowledgeUpdateServiceArray>("/rosplan_knowledge_base/update_array");
        query_knowledge_client_ = nh_.serviceClient<GetInstanceService>("/rosplan_knowledge_base/state/instances");
        ROS_INFO("(%s): Initalizing...", node_name.c_str());
        // auto tfBuffer = tf2_ros::Buffer();
        // tf2_ros::TransformListener tf_l(tfBuffer);
        ROS_INFO("(%s): Waiting for /pick_up_pose AS...", node_name.c_str());
        if (!pick_as.waitForServer(ros::Duration(20))) {
            ROS_ERROR("(%s): Could not connect to /pickup_pose AS", node_name.c_str());
            exit(0);
        }

        ROS_INFO("(%s): Setting publishers to torso and head controller...", node_name.c_str());
        torso_cmd = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
        head_cmd = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1);
        detected_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("/detected_aruco_pose", 1, true);

        ROS_INFO("(%s): Waiting for '/play_motion' AS...", node_name.c_str());
        if (!play_m_as.waitForServer(ros::Duration(20))) {
            ROS_ERROR("(%s): Could not connect to /play_motion AS", node_name.c_str());
            exit(0);
        }

        ROS_INFO("(%s): Connected!", node_name.c_str());
        ROS_INFO("(%s): Ready to receive.", node_name.c_str());

        service = nh_.advertiseService("my_pick_service", &PickUpAction::callback, this);
    }

    bool PickUpAction::callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        const rosplan_dispatch_msgs::ActionDispatch a;
        rosplan_dispatch_msgs::ActionDispatch::ConstPtr pa(new rosplan_dispatch_msgs::ActionDispatch());
        return concreteCallback(pa);
    }

    string PickUpAction::stripLeadingSlash(const string& s) {
        if (s[0] == '/') return std::move(s.substr(1));
        return std::move(s.substr());
    }

    void PickUpAction::prepareRobot() {
        ROS_INFO("(%s): Unfold arm safely", node_name.c_str());
        play_motion_msgs::PlayMotionGoal pmg;
        pmg.motion_name = "pregrasp";
        pmg.skip_planning = false;
        play_m_as.sendGoalAndWait(pmg);
        ROS_INFO("(%s): Done.", node_name.c_str());
        lowerHead();
        ROS_INFO("(%s): Robot prepared.", node_name.c_str());
    }

    void PickUpAction::lowerHead() {
        // ROS_INFO("(%s): Moving head down...", node_name.c_str());
        // trajectory_msgs::JointTrajectory jt;
        // jt.joint_names = { "head_1_joint", "head_2_joint" };
        // trajectory_msgs::JointTrajectoryPoint jtp;
        // jtp.positions = { 0.0, -0.75 };
        // jtp.time_from_start = ros::Duration(2.0);
        // jt.points.push_back(jtp);
        // head_cmd.publish(jt);
        // ROS_INFO("(%s): Lower head done.", node_name.c_str());
        ROS_INFO("(%s): Looking for aruco...", node_name.c_str());
        ros::ServiceClient look_around_client = nh_.serviceClient<std_srvs::Empty>("/look_around_interface/tiago_look_around");
        std_srvs::Empty e;
        look_around_client.call(e);
        ROS_INFO("(%s): Done.", node_name.c_str());
    }

    void PickUpAction::liftTorso() {
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
    bool PickUpAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // if (!robot_prepared) {
            prepareRobot();
            // robot_prepared = true;
        // }

        ROS_INFO("(%s): spherical_grasp_gui: Waiting for an aruco detection", node_name.c_str());
        auto aruco_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/aruco_single/pose");
        auto newFrameId = stripLeadingSlash(aruco_pose->header.frame_id);
        geometry_msgs::PoseStamped ps;
        ps.pose.position = aruco_pose->pose.position;
        // TODO: I dont know what is tfBuffer.get_latest_common_time(...), so I used ros::Time::now() instead
        ps.header.stamp = ros::Time::now();
        ps.header.frame_id = newFrameId;
        bool transformOK = false;
        geometry_msgs::PoseStamped aruco_ps;
        while (!transformOK && !ros::isShuttingDown()) {
            try {
                auto time = ros::Time::now();
                while (!tfBuffer.canTransform(newFrameId, string("base_footprint"), time)) ros::Duration(0.01).sleep();
                auto transform = tfBuffer.lookupTransform(string("base_footprint"), newFrameId, time);
                tf2::doTransform(ps, aruco_ps, transform);
                transformOK = true;
            }
            catch (exception e) {
                ROS_WARN("(%s): Exception on transforming point... trying again \n(%s)", node_name.c_str(), e.what());
                ros::Duration(0.01).sleep();
                ps.header.stamp = ros::Time::now();
            }
        }
        tiago_pick_demo::PickUpPoseGoal pick_g;
        ROS_INFO("(%s): Setting cube pose based on ArUco detection", node_name.c_str());
        pick_g.object_pose.pose.position = aruco_ps.pose.position;
        pick_g.object_pose.pose.position.z -= 0.1 * (1.0 / 2.0);

        ROS_INFO("(%s): aruco pose in base_footprint: %f, %f, %f", node_name.c_str(), pick_g.object_pose.pose.position.x, pick_g.object_pose.pose.position.y, pick_g.object_pose.pose.position.z);
        pick_g.object_pose.header.frame_id = "base_footprint";
        pick_g.object_pose.pose.orientation.w = 1.0;
        detected_pos_pub.publish(pick_g.object_pose);
        ROS_INFO("(%s): Gonna pick.", node_name.c_str());
        pick_as.sendGoalAndWait(pick_g);
        ROS_INFO("(%s): Done.", node_name.c_str());

        auto result = pick_as.getResult();
        if (result->error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("(%s): Failed to pick, not trying further.", node_name.c_str());
            return false;
        }

        // FIXME: 读取关节值来判断是否抓取成功是不可行的（因为始终会读取到程序设定的值而不是实际的值）
        // ros::ServiceClient gripper_controller_client = nh_.serviceClient<control_msgs::QueryTrajectoryState>("/gripper_controller/query_state");
        // control_msgs::QueryTrajectoryState para;
        // para.request.time = ros::Time::now();
        // gripper_controller_client.call(para);
        // for (int i = 0; i < para.response.name.size(); ++i) {
        //     ROS_INFO("(%s): %s: %f", node_name.c_str(), para.response.name[i].c_str(), para.response.position[i]);
        // }
        // double obj_width = nh_.param<double>("/pick_and_place_server/object_width", -1);
        // double obj_depth = nh_.param<double>("/pick_and_place_server/object_depth", -1);
        // if (para.response.position[0] + para.response.position[1] < min(obj_depth, obj_width)) {
        //     ROS_ERROR("(%s): Failed to grasp, pick up action failed.", node_name.c_str());
        //     return false;
        // }
        auto res = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/gripper_controller/state");
        ROS_INFO("(%s): actual[0]: %.4f, desired[0]: %.4f, actual[1]: %.4f, desired[1]: %.4f", node_name.c_str(), res->actual.positions[0], res->desired.positions[0], res->actual.positions[1], res->desired.positions[1]);
        // if (res->actual.positions[0] - res->desired.positions[0] + res->actual.positions[1] - res->desired.positions[1] < 0.01) {
        //     ROS_ERROR("(%s): Failed to grasp, pick up action failed.", node_name.c_str());
        //     ros::ServiceClient _plan_dispatch_cancel_client = nh_.serviceClient<std_srvs::Empty>("/rosplan_plan_dispatcher/cancel_dispatch");
        //     std_srvs::Empty empty;
        //     _plan_dispatch_cancel_client.call(empty);
        //     return false;
        // }

        liftTorso();
        ROS_INFO("(%s): Moving arm to a safe pose", node_name.c_str());
        play_motion_msgs::PlayMotionGoal pmg;
        pmg.motion_name = "pick_final_pose";
        pmg.skip_planning = false;
        play_m_as.sendGoalAndWait(pmg);
        ROS_INFO("(%s): Raise object done.", node_name.c_str());

        return true;
    }

    // rosplan_knowledge_msgs::KnowledgeItem createKnowledgeItem(uint8_t k_type, const string& attribute_name, )
} // close namespace

    /*-------------*/
    /* Main method */
    /*-------------*/

int main(int argc, char** argv) {

    ros::init(argc, argv, "pickup_action_interface");
    // ros::NodeHandle nh("~");
    // std::string actionserver;
    // nh.param("action_server", actionserver, std::string("/move_base"));
    // create PDDL action subscriber
    rosplane::PickUpAction rpti;

    rpti.runActionInterface();

    return 0;
}