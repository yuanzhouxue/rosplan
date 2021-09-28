#include "SimulatedMovebase.h"
#include <diagnostic_msgs/KeyValue.h>
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"

using namespace std;
using rosplan_knowledge_msgs::GetInstanceService;
using rosplan_knowledge_msgs::KnowledgeItem;
using rosplan_knowledge_msgs::KnowledgeUpdateService;
using rosplan_knowledge_msgs::KnowledgeUpdateServiceArray;

/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

    /* constructor */
    SimulatedMovebaseInterface::SimulatedMovebaseInterface(std::string &actionserver) : action_client_(actionserver, true) {
        // create a node handle to manage communication with ROS network
        ros::NodeHandle nh("~");
        // get waypoints reference frame from param server
        nh.param<std::string>("waypoint_frameid", waypoint_frameid_, "map");
        nh.param<std::string>("wp_namespace", wp_namespace_, "/rosplan_demo_waypoints/wp");
        nh.param<double>("probility", probility, 1.0);
        ROS_INFO("Simulated movebase with probility %lf", probility);
        // setup a move base clear costmap client (to be able to send clear costmap requests later on)
        clear_costmaps_client_ = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        update_knowledge_client_ = nh.serviceClient<KnowledgeUpdateService>("/rosplan_knowledge_base/update");
        update_knowledge_array_client_ = nh.serviceClient<KnowledgeUpdateServiceArray>("/rosplan_knowledge_base/update_array");
        query_knowledge_client_ = nh.serviceClient<GetInstanceService>("/rosplan_knowledge_base/state/instances");
    }

    bool SimulatedMovebaseInterface::wpIDtoPoseStamped(std::string wpID, geometry_msgs::PoseStamped& result) {
        ros::NodeHandle nh;
        std::vector<double> wp;
        if (nh.hasParam(wp_namespace_ + "/" + wpID)) {
            if (nh.getParam(wp_namespace_ + "/" + wpID, wp)) {
                if (wp.size() == 3) {
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
                } else {
                    ROS_ERROR("wp size must be equal to 3 : (x, y, and theta)");
                    return false;
                }
            }
        } else
            return false;
    }

    /* action dispatch callback */
    bool SimulatedMovebaseInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        {// // get waypoint ID from action dispatch msg
        // std::string wpID;
        // bool found = false;
        // // iterating over parameters (e.g. kenny, wp0, wp1)
        // for (size_t i = 0; i < msg->parameters.size(); i++) {
        //     // check their keys
        //     if (0 == msg->parameters[i].key.compare("to") or 0 == msg->parameters[i].key.compare("w1")) {
        //         // wp id found in msg params
        //         wpID = msg->parameters[i].value;
        //         found = true;
        //     }
        // }
        // if (!found) {
        //     ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?to", params.name.c_str());
        //     return false;
        // }

        // // get waypoint coordinates from its ID via query to parameter server
        // geometry_msgs::PoseStamped pose;
        // if (!wpIDtoPoseStamped(wpID, pose)) {
        //     ROS_ERROR("Waypoint not found in parameter server");
        //     return false;
        // }

        // ROS_INFO("KCL: (%s) waiting for move_base action server to start", params.name.c_str());
        // action_client_.waitForServer();

        // // dispatch MoveBase action
        // move_base_msgs::MoveBaseGoal goal;
        // goal.target_pose = pose;
        // action_client_.sendGoal(goal);

        // bool finished_before_timeout = action_client_.waitForResult();
        // if (finished_before_timeout) {

        //     actionlib::SimpleClientGoalState state = action_client_.getState();
        //     ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());

        //     if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {

        //         // publish feedback (achieved)
        //         return true;

        //     } else {

        //         // clear costmaps
        //         std_srvs::Empty emptySrv;
        //         clear_costmaps_client_.call(emptySrv);

        //         // publish feedback (failed)
        //         return false;
        //     }
        // } else {
        //     // timed out (failed)
        //     action_client_.cancelAllGoals();
        //     ROS_INFO("KCL: (%s) action timed out", params.name.c_str());
        //     return false;
        // }
        }
        double r = rand() / 2147483647.0;
        if (r > probility) {
            ROS_INFO("XYZ simulated movebase: (%s) failed", params.name.c_str());
            // get all the waypoints defined in KB
            std::vector<std::string> wps;
            GetInstanceService get_instance_srv;
            get_instance_srv.request.type_name = "waypoint";
            if (query_knowledge_client_.call(get_instance_srv)) {
                wps = get_instance_srv.response.instances;
            } else {
                ROS_ERROR("query knowledge failed");
            }
            // find a waypoint string name not shown to be added into KB
            string robot_at_wp_name = "wp";
            for (int i = wps.size(); i < wps.size() * 2; ++i) {
                bool shown = false;
                for (const auto& wp : wps) {
                    if (wp == robot_at_wp_name + to_string(i)) {
                        shown = true;
                        break;
                    }
                }
                if (!shown) {
                    robot_at_wp_name += to_string(i);
                    break;
                }
            }

            // add this wp to KB
            KnowledgeItem wp_item;
            wp_item.knowledge_type = KnowledgeItem::INSTANCE;
            wp_item.initial_time = ros::Time(0, 0);
            wp_item.instance_type = "waypoint";
            wp_item.instance_name = robot_at_wp_name;

            // add the fact that robot at this waypoint
            KnowledgeItem robot_at_wp_fact;
            robot_at_wp_fact.knowledge_type = KnowledgeItem::FACT;
            robot_at_wp_fact.initial_time = ros::Time(0, 0);
            robot_at_wp_fact.attribute_name = "robot_at";
            diagnostic_msgs::KeyValue kv1, kv2;
            kv1.key = "v";
            kv1.value = "kenny";
            kv2.key = "wp";
            kv2.value = robot_at_wp_name;
            robot_at_wp_fact.values = {kv1, kv2};
            
            vector<KnowledgeItem> to_be_updated = {wp_item, robot_at_wp_fact};

            // add the fact that this waypoint is connected with other wayponits
            for (const auto& wp : wps) {
                KnowledgeItem ki1, ki2;
                ki1.knowledge_type = ki2.knowledge_type = KnowledgeItem::FACT;
                ki1.initial_time = ki2.initial_time = ros::Time(0, 0);
                ki1.attribute_name =ki2.attribute_name = "connected";
                diagnostic_msgs::KeyValue kv1, kv2;
                kv1.key = "from";
                kv1.value = wp;
                kv2.key = "to";
                kv2.value = robot_at_wp_name;
                ki1.values = {kv1, kv2};
                swap(kv1.value, kv2.value);
                ki2.values = {kv1, kv2};
                to_be_updated.emplace_back(ki1);
                to_be_updated.emplace_back(ki2);
            }

            // TODO: add the distance between this waypoint with other wps

            KnowledgeUpdateServiceArray srv;
            srv.request.update_type = vector<uint8_t>(to_be_updated.size(), KnowledgeUpdateServiceArray::Request::ADD_KNOWLEDGE);
            srv.request.knowledge = move(to_be_updated);

            update_knowledge_array_client_.call(srv);
            return false;
        }
        ROS_INFO("XYZ simulated movebase: (%s) succeed", params.name.c_str());
        return true;
    }

    // rosplan_knowledge_msgs::KnowledgeItem createKnowledgeItem(uint8_t k_type, const string& attribute_name, )
} // close namespace

    /*-------------*/
    /* Main method */
    /*-------------*/

int main(int argc, char** argv) {

    ros::init(argc, argv, "simulated_movebase_action");
    ros::NodeHandle nh("~");
    std::string actionserver;
    nh.param("action_server", actionserver, std::string("/move_base"));
    // create PDDL action subscriber
    KCL_rosplan::SimulatedMovebaseInterface rpti(actionserver);

    rpti.runActionInterface();

    return 0;
}