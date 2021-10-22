#include "SensorInterface/GuestNotNearSensorInterface.h"

rosplane::GuestNotNearSensorInterface::GuestNotNearSensorInterface() : _nh("~") {
    pub = _nh.advertise<std_msgs::Bool>("guest_not_near", 1, false);
}

bool rosplane::GuestNotNearSensorInterface::concreteCallback(const rosplan_dispatch_msgs::SensorDispatch::ConstPtr& msg) {
    ROS_INFO("(SensorInterface: %s): concreteCallback", pred_name.c_str());
    auto scan1 = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan");
    ros::Duration(1.0).sleep();
    auto scan2 = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan");
    int n = scan1->ranges.size();
    float diff = 0.0;
    for (int i = 0; i < n; ++i) {
        diff += fabs(fmin(scan1->ranges[i], 1.0f) - fmin(scan2->ranges[i], 1.0f));
    }
    std_msgs::Bool boolMsg;
    boolMsg.data = diff <= 0.2;
    pub.publish(boolMsg);
    if (diff > 0.2) return false;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "guest_not_near_sensor_node");
    rosplane::GuestNotNearSensorInterface dsi;

    dsi.runSensorInterface();
    return 0;
}