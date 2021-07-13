#ifndef OBSERVE_ACTION_H
#define OBSERVE_ACTION_H
// ros header
#include <control_msgs/PointHeadActionGoal.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <rosplan_action_interface/RPActionInterface.h>

// opencv header
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace KCL_rosplan {
    class RPObserveAction : public RPActionInterface {
    public:
        RPObserveAction();
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

    private:
        void observeCallback(const sensor_msgs::ImageConstPtr& imgMsg);
        std::string imageTopic;
        ros::Publisher pub_head_topic;
        image_transport::Subscriber img_sub;


    };
}

#endif