#ifndef _LOOK_AROUND_H_
#define _LOOK_AROUND_H_

#include "look_around.h"

namespace rosplane {

    look_around::look_around(/* args */) : nh("~") {

        pub_head_topic = nh.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 5);
        pub_look_around_topic = nh.advertise<control_msgs::PointHeadActionGoal>("/head_controller/point_head_action/goal", 1);
        head_goal.joint_names.push_back("head_1_joint");
        head_goal.joint_names.push_back("head_2_joint");


        
    }


    bool look_around::tiagoCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        // head joint
        // 4 goals
        head_goal.points.resize(4);

        for (int i = 0; i < 4; ++i) {
            head_goal.points[i].positions.resize(2);
            head_goal.points[i].velocities.resize(2);
        }

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 2; ++j) {
                head_goal.points[i].velocities[j] = 0.0;
            }
        }
        // first goal
        head_goal.points[0].positions[0] = -1.2;
        head_goal.points[0].positions[1] = -0.7;
        head_goal.points[0].time_from_start = ros::Duration(2.0);

        // second goal
        head_goal.points[1].positions[0] = 1.2;
        head_goal.points[1].positions[1] = -0.7;
        head_goal.points[1].time_from_start = ros::Duration(8.0);

        // third goal
        head_goal.points[2].positions[0] = 1.2;
        head_goal.points[2].positions[1] = 0.0;
        head_goal.points[2].time_from_start = ros::Duration(10.0);

        // fouth goal
        head_goal.points[3].positions[0] = -1.2;
        head_goal.points[3].positions[1] = 0.0;
        head_goal.points[3].time_from_start = ros::Duration(16.0);

        head_goal.header.stamp = ros::Time::now();
        pub_head_topic.publish(head_goal);

        auto look_around_res = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/aruco_single/pose", ros::Duration(17.0));

        ROS_INFO("debug");
        if (look_around_res != nullptr) {
            // subscibe head pose
            try {
                ROS_INFO("%s", look_around_res->header.frame_id.c_str());
                look_around_res->pose.position
                listener.waitForTransform("/base_link", look_around_res->header.frame_id, ros::Time::now(), ros::Duration(1.0));
                listener.lookupTransform("/base_link", look_around_res->header.frame_id, ros::Time::now(), transform);

                
                after_look_around_goal.header.frame_id = "/base_link";
                after_look_around_goal.goal.max_velocity = 1.0;
                after_look_around_goal.goal.min_duration = ros::Duration(1.0);
                after_look_around_goal.goal.target.header.frame_id = "/base_link";
                after_look_around_goal.goal.pointing_axis.x = 1.0;
                after_look_around_goal.goal.pointing_frame = "/head_2_link";
                after_look_around_goal.goal.target.point.x = transform.getOrigin().x();
                after_look_around_goal.goal.target.point.y = transform.getOrigin().y();
                after_look_around_goal.goal.target.point.z = transform.getOrigin().z();
                pub_look_around_topic.publish(after_look_around_goal);
                return true;
            }
            catch (const std::exception& e) {
                // std::cerr << e.what() << '\n';
            }
        }
        return false;

    }

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "look_around_interface");
    ros::NodeHandle nh("~");
    rosplane::look_around lk;
    ros::ServiceServer service = nh.advertiseService("tiago_look_around", &rosplane::look_around::tiagoCallback, &lk);

    ros::spin();
    return 0;


}



#endif