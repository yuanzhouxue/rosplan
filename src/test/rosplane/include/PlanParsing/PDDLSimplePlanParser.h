/**
 * This file parses standard PDDL output and generates a list of ActionDispatch messages.
 */
#include "PlanParser.h"

#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "diagnostic_msgs/KeyValue.h"
#include "rosplane/ParsingStringSimple.h"

#include <string>
#include <sstream>

#ifndef rosplane_pddl_simple_plan_parser
#define rosplane_pddl_simple_plan_parser

namespace rosplane {

    class PDDLSimplePlanParser : public PlanParser {
    private:

        ros::ServiceClient get_operator_details_client;

    protected:

        /* post process plan */
        void processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch& msg, std::vector<std::string>& params);

        /* virtual methods */
        void reset();
        void preparePlan();
        void publishPlan();

    public:

        PDDLSimplePlanParser(ros::NodeHandle& nh);
        bool parsePlanFromString(rosplane::ParsingStringSimple::Request& req, rosplane::ParsingStringSimple::Response& res);
        ~PDDLSimplePlanParser();

        /* ROS interface */
        ros::Publisher plan_publisher;

    };
} // close namespace

#endif
