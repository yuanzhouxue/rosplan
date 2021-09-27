#include "DummySensorInterface.h"

bool rosplane::DummySensorInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
    return true;
}

int main(int argc, const char** argv) {
    return 0;
}