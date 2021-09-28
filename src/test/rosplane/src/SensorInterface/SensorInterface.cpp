#include "SensorInterface/SensorInterface.h"

/* The implementation of SensorInterface.h */
namespace rosplane {

    /* run action interface */
    void SensorInterface::runSensorInterface() {

        ros::NodeHandle nh("~");

        // knowledge base services
        std::string kb = "knowledge_base";
        nh.getParam("knowledge_base", kb);

        nh.getParam("pred_name", pred_name);

        // listen for action dispatch
        std::string sdt = "default_dispatch_topic";
        nh.getParam("sensor_dispatch_topic", sdt);
        std::string sft = "default_feedback_topic";
        nh.getParam("sensor_feedback_topic", sft);

        ros::SubscribeOptions ops;
        ops.template init<rosplane::SensorDispatch>(sdt, 1000, boost::bind(&rosplane::SensorInterface::dispatchCallback, this, _1));
        ops.transport_hints = ros::TransportHints();
        ops.allow_concurrent_callbacks = true;
        ros::Subscriber ds = nh.subscribe(ops); // nh.subscribe(adt, 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, this);

        sensor_feedback_pub = nh.advertise<rosplane::SensorFeedback>(sft, 1000, false);

        // loop
        ros::Rate loopRate(1);
        ros::AsyncSpinner spinner(4);
        spinner.start();

        ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());

        while (ros::ok()) {
            // pddl_action_parameters_pub.publish(params);
            loopRate.sleep();
        }
    }

    /* run action interface */
    void SensorInterface::dispatchCallback(const rosplane::SensorDispatch::ConstPtr& msg) {
        if (msg->name != pred_name) return;
        // call concrete implementation
        pred_hold = concreteCallback(msg);
        // ros::spinOnce();
        ROS_DEBUG("(%s): dispatchCallback end, going to pub feedback", ros::this_node::getName().c_str());
        rosplane::SensorFeedback feedback;
        feedback.hold = pred_hold;
        feedback.name = msg->name;
        sensor_feedback_pub.publish(feedback);
    }

} // close namespace
