#include "in_hand.h"

namespace ros_app {
    in_hand_SensorInterface::in_hand_SensorInterface() : _nh("~") {
        pub_head_topic = _nh.advertise<control_msgs::PointHeadActionGoal>("/head_controller/point_head_action/goal", 1);
        pub_arm_topic = _nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);
        imageTopic = "/xtion/rgb/image_raw";
        image_transport::TransportHints transportHints("compressed");
        image_transport::ImageTransport it(_nh);


        head_action_goal.header.frame_id = "/base_link";
        head_action_goal.goal.max_velocity = 1.0;
        head_action_goal.goal.min_duration = ros::Duration(1.0);
        head_action_goal.goal.target.header.frame_id = "/base_link";
        head_action_goal.goal.pointing_axis.x = 1.0;
        head_action_goal.goal.pointing_frame = "/head_2_link";

        arm_action_goal.joint_names.push_back("arm_1_joint");
        arm_action_goal.joint_names.push_back("arm_2_joint");
        arm_action_goal.joint_names.push_back("arm_3_joint");
        arm_action_goal.joint_names.push_back("arm_4_joint");
        arm_action_goal.joint_names.push_back("arm_5_joint");
        arm_action_goal.joint_names.push_back("arm_6_joint");
        arm_action_goal.joint_names.push_back("arm_7_joint");

        take_back_arm.joint_names.push_back("arm_1_joint");
        take_back_arm.joint_names.push_back("arm_2_joint");
        take_back_arm.joint_names.push_back("arm_3_joint");
        take_back_arm.joint_names.push_back("arm_4_joint");
        take_back_arm.joint_names.push_back("arm_5_joint");
        take_back_arm.joint_names.push_back("arm_6_joint");
        take_back_arm.joint_names.push_back("arm_7_joint");

        observe_flag = false;
    }

    void in_hand_SensorInterface::observeCallback(const sensor_msgs::ImageConstPtr& imgMsg) {
        cv_bridge::CvImagePtr cvImgPtr;
        cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
        cv::imshow("test", cvImgPtr->image);
        cv::waitKey(15);
    }

    bool in_hand_SensorInterface::concreteCallback(const rosplan_dispatch_msgs::SensorDispatch::ConstPtr& msg) {
        ROS_INFO("(%s): concretCallback", pred_name.c_str());
        auto res = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/arm_controller/state");

        arm_action_goal.points.resize(2);
        arm_action_goal.points[0].positions.resize(7);
        arm_action_goal.points[0].positions[0] = 0.28;
        arm_action_goal.points[0].positions[1] = -0.80;
        arm_action_goal.points[0].positions[2] = -3.22;
        arm_action_goal.points[0].positions[3] = 2.21;
        arm_action_goal.points[0].positions[4] = -0.41;
        arm_action_goal.points[0].positions[5] = -1.34;
        arm_action_goal.points[0].positions[6] = -2.07;
        arm_action_goal.points[0].velocities.resize(7);
        for (int i = 0; i < 7; i++) {
            arm_action_goal.points[0].velocities[i] = 0.0;
        }
        arm_action_goal.points[0].time_from_start = ros::Duration(4.0);

        arm_action_goal.points[1].positions.resize(7);
        arm_action_goal.points[1].positions[0] = 0.28;
        arm_action_goal.points[1].positions[1] = -0.80;
        arm_action_goal.points[1].positions[2] = -3.22;
        arm_action_goal.points[1].positions[3] = 2.21;
        arm_action_goal.points[1].positions[4] = -0.41;
        arm_action_goal.points[1].positions[5] = -1.34;
        arm_action_goal.points[1].positions[6] = 2.0;
        arm_action_goal.points[1].velocities.resize(7);
        for (int i = 0; i < 7; i++) {
            arm_action_goal.points[1].velocities[i] = 0.0;
        }
        arm_action_goal.points[1].time_from_start = ros::Duration(8.0);
        arm_action_goal.header.stamp = ros::Time::now();
        pub_arm_topic.publish(arm_action_goal);
     
        geometry_msgs::PoseStamped ps;
        ros::Time time;
        std::string err_string;
        bool transform_ok = false;
        ps.header.frame_id = "/gripper_link";
        ps.header.stamp = ros::Time(listener.getLatestCommonTime("/base_link", "/gripper_link", time, &err_string));

        while (!ros::isShuttingDown() && !transform_ok) {
            try {
                listener.waitForTransform("/base_link", "/gripper_link", ros::Time::now(), ros::Duration(1.0));
                listener.lookupTransform("/base_link", "/gripper_link", ros::Time::now(), transform);
                transform_ok = true;
            }
            catch (const std::exception& e) {
            }
        }
        head_action_goal.goal.target.point.x = transform.getOrigin().x();
        head_action_goal.goal.target.point.y = transform.getOrigin().y();
        head_action_goal.goal.target.point.z = transform.getOrigin().z();
        pub_head_topic.publish(head_action_goal);
        ros::Duration(2.0).sleep();

        auto observe_res = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/aruco_single/pose", ros::Duration(8.0));
        auto now = ros::Time::now();
        if (observe_res != nullptr && now - observe_res->header.stamp < ros::Duration(2.0)) {
            ROS_INFO("%d, %d", observe_res->header.stamp.nsec, observe_res->header.stamp.sec);
            ROS_INFO("%d, %d", now.nsec, now.sec);
            observe_flag = true;
        }

        take_back_arm.points.resize(1);
        take_back_arm.points[0].positions.resize(7);
        take_back_arm.points[0].positions[0] = 0.59;
        take_back_arm.points[0].positions[1] = 0.09;
        take_back_arm.points[0].positions[2] = -1.38;
        take_back_arm.points[0].positions[3] = 2.29;
        take_back_arm.points[0].positions[4] = 0.21;
        take_back_arm.points[0].positions[5] = 1.11;
        take_back_arm.points[0].positions[6] = 0.0;
        take_back_arm.points[0].velocities.resize(7);
        for (int i = 0; i < 7; i++) {
            take_back_arm.points[0].velocities[i] = 0.0;
        }
        take_back_arm.points[0].time_from_start = ros::Duration(12.0);
        take_back_arm.header.stamp = ros::Time::now();
        pub_arm_topic.publish(take_back_arm);
        
        if (observe_flag) {
            return observe_flag;
        }
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "inhand_predicate_observe");
    ros_app::in_hand_SensorInterface ipo;
    ipo.runSensorInterface();
    return 0;
}