#include "robot_at.h"

namespace ros_app {
    robot_at_SensorInterface::robot_at_SensorInterface() : _nh("~") {
        pub_head_topic = _nh.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 5);
        pub_look_around_topic = _nh.advertise<control_msgs::PointHeadActionGoal>("/head_controller/point_head_action/goal", 1);
        head_goal.joint_names.push_back("head_1_joint");
        head_goal.joint_names.push_back("head_2_joint");
    }

    bool robot_at_SensorInterface::concreteCallback(const rosplan_dispatch_msgs::SensorDispatch::ConstPtr& msg) {
        return true;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_at_predicate_observation");
    ros::NodeHandle nh("~");
    ros_app::robot_at_SensorInterface rapo;
    rapo.runSensorInterface();
    return 0;
}