#include "observe_action.h"

namespace KCL_rosplan {

    RPObserveAction::RPObserveAction() {
        ros::NodeHandle nh("~");
        pub_head_topic = nh.advertise<control_msgs::PointHeadActionGoal>("/head_controller/point_head_action/goal", 1);
        imageTopic = "/xtion/rgb/image_raw";
        image_transport::TransportHints transportHints("compressed");
        image_transport::ImageTransport it(nh);
        img_sub = it.subscribe(imageTopic, 1, &RPObserveAction::observeCallback, this, transportHints);
        // ros::spin();
        // cv::destroyWindow("test");
    }

    void RPObserveAction::observeCallback(const sensor_msgs::ImageConstPtr& imgMsg) {
        cv_bridge::CvImagePtr cvImgPtr;
        cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
        cv::imshow("test", cvImgPtr->image);
        cv::waitKey(15);
    }

    bool RPObserveAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // ros nodehandle init
        ros::NodeHandle nh;


        // tf listener and stampedTransform
        tf::TransformListener listener;
        tf::StampedTransform transform;

        // move head to look hand


        // msg init
        control_msgs::PointHeadActionGoal goal;
        goal.header.frame_id = "/base_link";
        goal.goal.max_velocity = 1.0;
        goal.goal.min_duration = ros::Duration(1.0);
        goal.goal.target.header.frame_id = "/base_link";
        goal.goal.pointing_axis.x = 1.0;
        goal.goal.pointing_frame = "/head_2_link";


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
                std::cerr << e.what() << '\n';
            }
        }
        goal.goal.target.point.x = transform.getOrigin().x();
        goal.goal.target.point.y = transform.getOrigin().y();
        goal.goal.target.point.z = transform.getOrigin().z();
        pub_head_topic.publish(goal);

        

        // get the image data

        // cv::namedWindow("test", cv::WINDOW_AUTOSIZE);

        // cv_bridge::CvImagePtr cvImgMsg;


        return true;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "observe_action_interface");
    KCL_rosplan::RPObserveAction oa;
    oa.runActionInterface();
    return 0;
}


