#include <RPActionInterface.h>

namespace rosplane {
    class DummyActionInterface : public RPActionInterface {
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}