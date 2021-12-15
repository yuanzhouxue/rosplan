#include <SensorInterface/PhotoTakenSensor.h>

using namespace std;

namespace turtlebot3_navigation_demo {
    PhotoTakenSensor::PhotoTakenSensor() : _nh("~"){
        photo_store_path = _nh.param<std::string>("image_path", "~/Pictures/");
    }

    bool PhotoTakenSensor::concreteCallback(const rosplan_dispatch_msgs::SensorDispatchConstPtr& msg) {
        std::string fullpath = photo_store_path;
        for (const auto& kv : msg->typed_parameters) {
            if (kv.key == "wp") {
                fullpath += kv.value + ".png";
                break;
            }
        }
        boost::system::error_code err;
        auto file_status = boost::filesystem::status(fullpath, err);
        if (err) return false;
        if (!boost::filesystem::exists(file_status)) return false;
        if (boost::filesystem::is_directory(file_status)) return false;

        cv::Mat m = cv::imread(fullpath);
        if (m.cols != 640 || m.rows != 480) return false;
        return true;
    }
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "photo_taken_sensor");
    turtlebot3_navigation_demo::PhotoTakenSensor vsi;
    vsi.runSensorInterface();
    return 0;
}
