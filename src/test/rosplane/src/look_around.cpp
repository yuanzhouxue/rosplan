#ifndef _LOOK_AROUND_H_
#define _LOOK_AROUND_H_

// ros header
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <control_msgs/PointHeadActionGoal.h>
// aruco header
#include <aruco/aruco.h>


bool tiagoCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    ros::NodeHandle nh;
    ros::Publisher pub_head_topic = nh.advertise<trajectory_msgs::JointTrajectory>("/trajectory_msgs/command", 5);
    ros::Publisher pub_look_around_topic = nh.advertise<control_msgs::PointHeadActionGoal>("/head_controller/point_head_action/goal", 1);
    
    trajectory_msgs::JointTrajectory head_goal;
    control_msgs::PointHeadActionGoal after_look_around_goal;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    // head joint
    head_goal.joint_names.push_back("head_1_joint");
    head_goal.joint_names.push_back("head_2_joint");
    

    // 4 goals
    head_goal.points.resize(4);

    for (int i = 0; i < 4; ++i){
        head_goal.points[i].positions.resize(2);
        head_goal.points[i].velocities.resize(2);
    }
    
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 2; ++j){
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
    head_goal.points[1].time_from_start = ros::Duration(4.0);
    
    // third goal
    head_goal.points[2].positions[0] = 1.2;
    head_goal.points[2].positions[1] = 0.0;
    head_goal.points[2].time_from_start = ros::Duration(6.0);
    
    // fouth goal
    head_goal.points[3].positions[0] = -1.2;
    head_goal.points[3].positions[1] = 0.0;
    head_goal.points[3].time_from_start = ros::Duration(8.0);
    
    head_goal.header.stamp = ros::Time::now() + ros::Duration(8.0);

    std::cout << "debug2" << std::endl;
    pub_head_topic.publish(head_goal);

    auto look_around_res = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/aruco_single/pose");
    ROS_INFO("%s", look_around_res->header.frame_id);
    if (look_around_res != nullptr){
        // subscibe head pose
        try
        {
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
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
        }    
    }
    return false;
    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "look_around_interface");
    ros::NodeHandle nh("~");
    ros::ServiceServer service = nh.advertiseService("tiago_look_around", tiagoCallback);
    
    ros::spin();
    return 0;


}



#endif