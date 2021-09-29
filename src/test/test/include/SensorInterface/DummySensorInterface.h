#include <SensorInterface/SensorInterface.h>

namespace rosplane {
    class DummySensorInterface : public SensorInterface {
        bool concreteCallback(const rosplane::SensorDispatch::ConstPtr& msg);
    };
}