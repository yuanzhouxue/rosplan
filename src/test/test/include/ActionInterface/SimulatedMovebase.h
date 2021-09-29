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

#ifndef SIMULATED_MOVEBASE_H
#define SIMULATED_MOVEBASE_H

/**
 * This file defines an action interface created in tutorial 10.
 */
namespace KCL_rosplan {

    class SimulatedMovebaseInterface : public RPActionInterface {

    private:
        // action lib client that will make the communication with move_base action server
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;

        // to clear costmaps if move base gets stuck
        ros::ServiceClient clear_costmaps_client_;
        ros::ServiceClient update_knowledge_client_;
        ros::ServiceClient update_knowledge_array_client_;
        ros::ServiceClient query_knowledge_client_;


        /// nodehandle, manages e.g. subscriptions and publications
        ros::NodeHandle nh_;

        /// waypoints reference frame
        std::string waypoint_frameid_;
        std::string wp_namespace_;
        double probility;

    public:

        /* constructor */
        SimulatedMovebaseInterface(std::string &actionserver);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

        /**
         * @brief query wp coordinates from parameter server, where the waypoint values (x, y, theta) are assumed to be stored
         * @param wpID an integer with the waypoint id from which to fetch the pose values (x, y, theta)
         * @param result return value gets written here by reference
         * @return true if waypoint values were found in parameter server, false otherwise
        */
        bool wpIDtoPoseStamped(std::string wpID, geometry_msgs::PoseStamped& result);
    };
}
#endif