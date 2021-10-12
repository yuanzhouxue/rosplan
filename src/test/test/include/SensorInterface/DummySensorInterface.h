#include <rosplan_sensor_interface/SensorInterface.h>

namespace rosplane {
    class DummySensorInterface : public KCL_rosplan::SensorInterface {
        bool concreteCallback(const rosplan_dispatch_msgs::SensorDispatch::ConstPtr& msg);
    };
}