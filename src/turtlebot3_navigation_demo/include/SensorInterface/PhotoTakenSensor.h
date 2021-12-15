#include <rosplan_sensor_interface/SensorInterface.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <geometry_msgs/Twist.h>
#include <turtlebot3_msgs/Sound.h>
#include <boost/filesystem.hpp>

namespace turtlebot3_navigation_demo {
    class PhotoTakenSensor : public KCL_rosplan::SensorInterface {
        ros::NodeHandle _nh;
        std::string photo_store_path;
        bool concreteCallback(const rosplan_dispatch_msgs::SensorDispatchConstPtr& msg);
    public:
        PhotoTakenSensor();
    };
}