#include <rosplan_action_interface/RPActionInterface.h>

namespace rosplane {
    class DummyActionInterface : public KCL_rosplan::RPActionInterface {
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}