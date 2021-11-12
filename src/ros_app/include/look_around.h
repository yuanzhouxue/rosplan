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

namespace ros_app {
    class look_around {
    private:
        /* data */
        ros::NodeHandle _nh;
        ros::Publisher pub_head_topic;
        ros::Publisher pub_look_around_topic;
        trajectory_msgs::JointTrajectory head_goal;
        control_msgs::PointHeadActionGoal after_look_around_goal;
        tf::TransformListener listener;
        tf::StampedTransform transform;

    public:
        bool tiagoCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
        look_around(/* args */);
    };

}