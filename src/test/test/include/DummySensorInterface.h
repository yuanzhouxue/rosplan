#include <SensorInterface/SensorInterface.h>

namespace rosplane {
    class DummySensorInterface : public SensorInterface {
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}