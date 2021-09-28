#include <ros/ros.h>
#include <boost/tokenizer.hpp>
#include <std_srvs/Empty.h>

#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_knowledge_msgs/DomainFormula.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include "rosplane/SensorDispatch.h"
#include "rosplane/SensorFeedback.h"
#include "diagnostic_msgs/KeyValue.h"

#ifndef rosplane_sensor_interface
#define rosplane_sensor_interface

/**
 * This file defines the RPActionInterface header.
 * This header defines a standard action interface for ROSPlan.
 * This interface will link a PDDL action to some implementation, most
 * commonly as an actionlib client.
 */
namespace rosplane {

    class SensorInterface {
        
    protected:

        std::string pred_name;

        ros::Publisher pddl_action_parameters_pub;

        /* action feedback to planning system */
        ros::Publisher sensor_feedback_pub;

        /* service handle to PDDL knowledge base */
        ros::ServiceClient update_knowledge_client;

        /* pred status */
        bool pred_hold;

    public:

        /* main loop for action interface */
        void runSensorInterface();

        /* listen to and process action_dispatch topic */
        void dispatchCallback(const rosplane::SensorDispatch::ConstPtr& msg);

        /* perform or call real action implementation */
        virtual bool concreteCallback(const rosplane::SensorDispatch::ConstPtr& msg) = 0;
    };
}
#endif
