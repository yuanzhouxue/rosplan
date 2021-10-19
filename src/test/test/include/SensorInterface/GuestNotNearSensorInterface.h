#include <rosplan_sensor_interface/SensorInterface.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <cmath>

namespace rosplane {
    class GuestNotNearSensorInterface : public KCL_rosplan::SensorInterface {
        ros::NodeHandle _nh;
        ros::Publisher pub;
        bool concreteCallback(const rosplan_dispatch_msgs::SensorDispatch::ConstPtr& msg);
    public:
        GuestNotNearSensorInterface();
    };
}