#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rosplan_action_interface/RPActionInterface.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace turtlebot3_navigation_demo {

    class TakePhotoAction : public KCL_rosplan::RPActionInterface {

    public:
        TakePhotoAction();
        void imageCallback(const sensor_msgs::ImageConstPtr& img);
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

    private:
        ros::NodeHandle _nh;
        image_transport::ImageTransport _it;
        image_transport::Subscriber imgSub;
        /// waypoints reference frame
        std::string photo_store_path;
        cv::Mat mat;
    };
}

