#include <SensorInterface/VisitedSensor.h>

bool turtlebot3_navigation_demo::VisitedSensorInterface::concreteCallback(const rosplan_dispatch_msgs::SensorDispatchConstPtr& msg) {
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "");
    /* code */
    return 0;
}
