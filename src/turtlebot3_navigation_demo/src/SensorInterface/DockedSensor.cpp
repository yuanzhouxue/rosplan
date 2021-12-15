#include <SensorInterface/DockedSensor.h>

using namespace std;

namespace turtlebot3_navigation_demo {
    DockedSensor::DockedSensor() : _nh("~"), _it(_nh) {
        string image_topic = _nh.param<string>("image_topic", "/raspicam_node/image");
        image_transport::TransportHints hints("compressed");
        imgSub = _it.subscribe(image_topic, 1, &DockedSensor::imageCallback, this, hints);
        markerImagePub = _it.advertise("/marker_image", 1);
        string vel_topic = _nh.param<string>("cmd_vel_topic", "/cmd_vel");
        cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>(vel_topic, 1);
        sound_pub = _nh.advertise<turtlebot3_msgs::Sound>("/sound", 1);
    }


    void DockedSensor::imageCallback(const sensor_msgs::ImageConstPtr& img) {
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        // arucoFound = detectAruco(imgPtr->image);
    }


    bool DockedSensor::detectAruco(const cv::Mat& img) {
        // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
        // cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        // vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;
        // vector<int> markerIds;
        // cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        // sensor_msgs::ImagePtr msg;
        // if (markerIds.size()) {
        //     cv::Mat markerImage = img.clone();
        //     cv::aruco::drawDetectedMarkers(markerImage, markerCorners);
        //     msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", markerImage).toImageMsg();
        // } else {
        //     msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        // }
        // markerImagePub.publish(msg);
        // return markerIds.size() > 0;
        return true;
    }

    bool DockedSensor::concreteCallback(const rosplan_dispatch_msgs::SensorDispatchConstPtr& msg) {
        return true;
    }
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "docked_sensor");
    turtlebot3_navigation_demo::DockedSensor vsi;
    vsi.runSensorInterface();
    return 0;
}
