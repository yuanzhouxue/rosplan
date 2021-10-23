#include <rosplan_sensor_interface/SensorInterface.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <geometry_msgs/Twist.h>

namespace turtlebot3_navigation_demo {
    class VisitedSensorInterface : public KCL_rosplan::SensorInterface {
        ros::NodeHandle _nh;
        image_transport::ImageTransport _it;
        image_transport::Subscriber imgSub;
        image_transport::Publisher markerImagePub;
        ros::Subscriber cam_info_sub;
        ros::Publisher cmd_vel_pub;
        bool arucoFound;
        bool rotateRobot(float angular_vel);
        bool detectAruco(const cv::Mat& img);
        void imageCallback(const sensor_msgs::ImageConstPtr& img);
        bool concreteCallback(const rosplan_dispatch_msgs::SensorDispatchConstPtr& msg);
    public:
        VisitedSensorInterface();
    };
}