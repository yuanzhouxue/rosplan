#include "ActionInterface/TakePhotoAction.h"

namespace turtlebot3_navigation_demo {

    // constructor
    TakePhotoAction::TakePhotoAction() : _nh("~"), _it(_nh) {
        std::string image_topic = _nh.param<std::string>("image_topic", "/raspicam_node/image");
        photo_store_path = _nh.param<std::string>("image_path", "~/Pictures/");
        image_transport::TransportHints hints("compressed");
        imgSub = _it.subscribe(image_topic, 1, &TakePhotoAction::imageCallback, this, hints);
    }

    void TakePhotoAction::imageCallback(const sensor_msgs::ImageConstPtr& img) {
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        this->mat = imgPtr->image;
    }

    // action dispatch callback
    bool TakePhotoAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        for (const auto& kv : msg->parameters) {
            if (kv.key == "wp") {
                cv::imwrite(photo_store_path + kv.value + ".png", this->mat);
                break;
            }
        }
        return true;
    }
} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "take_photo_action");
    // create PDDL action subscriber
    turtlebot3_navigation_demo::TakePhotoAction tpa;
    tpa.runActionInterface();

    return 0;
}
