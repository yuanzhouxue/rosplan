#include <rosplan_sensor_interface/SensorInterface.h>

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

namespace rosplane
{
    class RobotAtPredicateObservation : public KCL_rosplan::SensorInterface
    {
    private:
        ros::NodeHandle nh;
        ros::Publisher pub_head_topic;
        ros::Publisher pub_look_around_topic;
        trajectory_msgs::JointTrajectory head_goal;
        control_msgs::PointHeadActionGoal after_look_around_goal;
        tf::TransformListener listener;
        tf::StampedTransform transform;
    public:
        bool concreteCallback(const rosplan_dispatch_msgs::SensorDispatch::ConstPtr& msg);
        RobotAtPredicateObservation(/* args */);
    };
}
