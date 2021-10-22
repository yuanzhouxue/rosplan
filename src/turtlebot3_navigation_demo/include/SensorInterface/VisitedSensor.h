#include <rosplan_sensor_interface/SensorInterface.h>

namespace turtlebot3_navigation_demo {
    class VisitedSensorInterface : KCL_rosplan::SensorInterface {
        bool concreteCallback(const rosplan_dispatch_msgs::SensorDispatchConstPtr& msg);
    };
}