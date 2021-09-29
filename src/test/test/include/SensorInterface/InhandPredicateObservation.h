#include <SensorInterface/SensorInterface.h>

// ros header
#include <control_msgs/PointHeadActionGoal.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <RPActionInterface.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>

// aruco header
#include <aruco/aruco.h>

// opencv header
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
namespace rosplane {
    class InhandPredicateObervation : public SensorInterface {
    public:
        bool concreteCallback(const rosplane::SensorDispatch::ConstPtr& msg);
        InhandPredicateObervation();


    private:
        void observeCallback(const sensor_msgs::ImageConstPtr& imgMsg);

        std::string imageTopic;
        std::string windowName;
        std::string armTopic;

        ros::Publisher pub_head_topic;
        ros::Publisher pub_arm_topic;

        ros::Subscriber sub_arm_topic;

        ros::ServiceServer service;

        image_transport::Subscriber img_sub;
        tf::TransformListener listener;
        tf::StampedTransform transform;

        control_msgs::PointHeadActionGoal head_action_goal;
        trajectory_msgs::JointTrajectory arm_action_goal;
    };
}
