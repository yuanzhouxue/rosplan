#include <SensorInterface/VisitedSensor.h>

using namespace std;

namespace turtlebot3_navigation_demo {
    VisitedSensorInterface::VisitedSensorInterface() : _nh("~"), _it(_nh) {
        string image_topic = _nh.param<string>("image_topic", "/raspicam_node/image");
        image_transport::TransportHints hints("compressed");
        imgSub = _it.subscribe(image_topic, 1, &VisitedSensorInterface::imageCallback, this, hints);
        markerImagePub = _it.advertise("/marker_image", 1);
        string vel_topic = _nh.param<string>("cmd_vel_topic", "/cmd_vel");
        cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>(vel_topic, 1);
        sound_pub = _nh.advertise<turtlebot3_msgs::Sound>("/sound", 1);
    }

    bool VisitedSensorInterface::rotateRobot(float angular_vel) {
        geometry_msgs::Twist twist;
        twist.angular.z = angular_vel;
        cmd_vel_pub.publish(twist);
    }

    void VisitedSensorInterface::imageCallback(const sensor_msgs::ImageConstPtr& img) {
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        arucoFound = detectAruco(imgPtr->image);
    }


    bool VisitedSensorInterface::detectAruco(const cv::Mat& img) {
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;
        vector<int> markerIds;
        cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        sensor_msgs::ImagePtr msg;
        if (markerIds.size()) {
            cv::Mat markerImage = img.clone();
            cv::aruco::drawDetectedMarkers(markerImage, markerCorners);
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", markerImage).toImageMsg();
        } else {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        }
        markerImagePub.publish(msg);
        return markerIds.size() > 0;
    }

    bool VisitedSensorInterface::concreteCallback(const rosplan_dispatch_msgs::SensorDispatchConstPtr& msg) {
        ros::Time start = ros::Time::now();
        bool found = false;
        turtlebot3_msgs::Sound s;
        s.value = turtlebot3_msgs::Sound::ON;
        // 每次转动45度
        float eachTurnInPi = 0.25f;
        int turnRounds = 2.15 / eachTurnInPi;
        if (arucoFound) {
            return true;
        }
        for (int _ = 0; _ < turnRounds; ++_) {
            rotateRobot(-0.5f);
            ros::Duration(2 * 3.141592653589 * eachTurnInPi).sleep();
            rotateRobot(0.0f);
            ros::Duration(0.5).sleep();
            if (arucoFound) {
                // sound_pub.publish(s);
                found = true;
                break;
            }
        }
        rotateRobot(0.0f);
        return found;
    }
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "visited_sensor");
    turtlebot3_navigation_demo::VisitedSensorInterface vsi;
    vsi.runSensorInterface();
    return 0;
}
