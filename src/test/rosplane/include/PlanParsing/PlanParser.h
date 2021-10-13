/**
 * This file parses the output of a planner and generates a list of ActionDispatch messages.
 */
#include "ros/ros.h"

#include <string>
#include <fstream>

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "rosplan_planning_msgs/ParsingService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"

#ifndef rosplane_plan_parser
#define rosplane_plan_parser

namespace rosplane {

    class PlanParser {
    private:

    protected:

        ros::NodeHandle* node_handle;

        /* plan subscription */
        std::string planner_output;
        bool planner_output_received;
        double planner_output_time;

        std::vector<rosplan_dispatch_msgs::ActionDispatch> action_list;

        virtual void reset() = 0;
        virtual void preparePlan() = 0;
        virtual void publishPlan() = 0;

    public:

        void plannerCallback(const std_msgs::String& plannerOutput);

        bool parsePlanFromFile(rosplan_planning_msgs::ParsingService::Request& req, rosplan_planning_msgs::ParsingService::Response& res);
        bool parsePlan(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    };
} // close namespace

#endif
