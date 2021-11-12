#include <rosplan_sensor_interface/SensorInterface.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <cmath>

namespace ros_app {
    class guest_not_near_SensorInterface : public KCL_rosplan::SensorInterface {
        ros::NodeHandle _nh;
        ros::Publisher pub;
        bool concreteCallback(const rosplan_dispatch_msgs::SensorDispatch::ConstPtr& msg);
    public:
        guest_not_near_SensorInterface();
    };
}