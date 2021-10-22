#include "SensorInterface/DummySensorInterface.h"

bool rosplane::DummySensorInterface::concreteCallback(const rosplan_dispatch_msgs::SensorDispatch::ConstPtr& msg) {
    ROS_INFO("(SensorInterface: %s): concreteCallback", pred_name.c_str());
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dummysensor_node");
    rosplane::DummySensorInterface dsi;

    dsi.runSensorInterface();
    return 0;
}