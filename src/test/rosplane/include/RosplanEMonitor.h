#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/DispatchService.h"

#ifndef ROSPLANE_MONITOR_H
#define ROSPLANE_MONITOR_H

namespace rosplane {
    class ROSPlanEMonitor {
    private:
        ros::ServiceClient _problem_generation_client;
        ros::ServiceClient _plan_client;
        ros::ServiceClient _parse_client;
        ros::ServiceClient _plan_dispatch_client;
        ros::ServiceClient _plan_dispatch_cancel_client;
    public:
        ROSPlanEMonitor();
        void action_feedback_callback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);
    };
}

#endif // !ROSPLANE_MONITOR_H