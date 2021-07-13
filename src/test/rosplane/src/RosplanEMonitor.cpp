#include "RosplanEMonitor.h"

namespace rosplane {
    ROSPlanEMonitor::ROSPlanEMonitor() {
        ros::NodeHandle nh("~");
        _problem_generation_client = nh.serviceClient<std_srvs::Empty>("/rosplan_problem_interface/problem_generation_server");
        _plan_client = nh.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server");
        _parse_client = nh.serviceClient<std_srvs::Empty>("/rosplan_parsing_interface/parse_plan");
        _plan_dispatch_client = nh.serviceClient<rosplan_dispatch_msgs::DispatchService>("/rosplan_plan_dispatcher/dispatch_plan");
        _plan_dispatch_cancel_client = nh.serviceClient<std_srvs::Empty>("/rosplan_plan_dispatcher/cancel_dispatch");
    }
    void ROSPlanEMonitor::action_feedback_callback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {
        // TODO: use thread or process
        ROS_WARN("(%s): Action feedback.", ros::this_node::getName().c_str());
        if (msg->status == rosplan_dispatch_msgs::ActionFeedback::ACTION_FAILED) {
            ROS_INFO("(%s): Action failed.", ros::this_node::getName().c_str());
            ROS_ERROR("Rosplane replanning");
            std_srvs::Empty empty;
            _plan_dispatch_cancel_client.call(empty);
            _problem_generation_client.call(empty);
            _plan_client.call(empty);
            _parse_client.call(empty);
            rosplan_dispatch_msgs::DispatchService srv;
            _plan_dispatch_client.call(srv);
        }
    }
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "rosplane_monitor");
    ros::NodeHandle nh("~");
    rosplane::ROSPlanEMonitor monitor;
    ros::Subscriber sub = nh.subscribe<rosplan_dispatch_msgs::ActionFeedback>("/rosplan_plan_dispatcher/action_feedback", 10, &rosplane::ROSPlanEMonitor::action_feedback_callback, &monitor);
    ros::spin();
    return 0;
}