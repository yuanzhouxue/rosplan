#include "ActionInterface/observe_action.h"
#include <diagnostic_msgs/KeyValue.h>
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"

using namespace std;
using rosplan_knowledge_msgs::GetInstanceService;
using rosplan_knowledge_msgs::KnowledgeItem;
using rosplan_knowledge_msgs::KnowledgeUpdateService;
using rosplan_knowledge_msgs::KnowledgeUpdateServiceArray;

namespace rosplane {

    RPObserveAction::RPObserveAction() {
        ros::NodeHandle nh("~");
        pub_head_topic = nh.advertise<control_msgs::PointHeadActionGoal>("/head_controller/point_head_action/goal", 1);
        pub_arm_topic = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);
        imageTopic = "/xtion/rgb/image_raw";
        image_transport::TransportHints transportHints("compressed");
        image_transport::ImageTransport it(nh);
        img_sub = it.subscribe(imageTopic, 1, &RPObserveAction::observeCallback, this, transportHints);
        service = nh.advertiseService("my_observe_service", &RPObserveAction::serviceCallback, this);

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

        update_knowledge_array_client_ = nh.serviceClient<KnowledgeUpdateServiceArray>("/rosplan_knowledge_base/update_array");
    }

    bool RPObserveAction::serviceCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
        const rosplan_dispatch_msgs::ActionDispatch a;
        rosplan_dispatch_msgs::ActionDispatch::ConstPtr pa(new rosplan_dispatch_msgs::ActionDispatch());
        return concreteCallback(pa);
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

        // move head to look hand        
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
                // std::cerr << e.what() << '\n';
            }
        }
        head_action_goal.goal.target.point.x = transform.getOrigin().x();
        head_action_goal.goal.target.point.y = transform.getOrigin().y();
        head_action_goal.goal.target.point.z = transform.getOrigin().z();
        pub_head_topic.publish(head_action_goal);
        // TODO: ???????????????????????????????????????????????????????????????
        // ??????sleep????????????????????????ros???????????????
        ros::Duration(3.0).sleep();



        // rotate wrist test
        // get the arms' position
        auto res = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/arm_controller/state");

        // 

        // first position
        arm_action_goal.points.resize(2);
        arm_action_goal.points[0].positions.resize(7);
        arm_action_goal.points[0].positions[0] = res->actual.positions[0];
        arm_action_goal.points[0].positions[1] = -0.21;
        arm_action_goal.points[0].positions[2] = -2.20;
        arm_action_goal.points[0].positions[3] = 1.16;
        arm_action_goal.points[0].positions[4] = -0.71;
        arm_action_goal.points[0].positions[5] = 1.11;
        arm_action_goal.points[0].positions[6] = 2.0;
        arm_action_goal.points[0].velocities.resize(7);
        for (int i = 0; i < 7; i++) {
            arm_action_goal.points[0].velocities[i] = 0.0;
        }

        arm_action_goal.points[0].time_from_start = ros::Duration(4.0);

        // second position
        arm_action_goal.points[1].positions.resize(7);
        arm_action_goal.points[1].positions[0] = res->actual.positions[0];
        arm_action_goal.points[1].positions[1] = -0.21;
        arm_action_goal.points[1].positions[2] = -2.20;
        arm_action_goal.points[1].positions[3] = 1.16;
        arm_action_goal.points[1].positions[4] = -0.71;
        arm_action_goal.points[1].positions[5] = 1.11;
        arm_action_goal.points[1].positions[6] = -2.0;
        arm_action_goal.points[1].velocities.resize(7);
        for (int i = 0; i < 7; i++) {
            arm_action_goal.points[1].velocities[i] = 0.0;
        }

        arm_action_goal.points[1].time_from_start = ros::Duration(8.0);
        arm_action_goal.header.stamp = ros::Time::now();
        pub_arm_topic.publish(arm_action_goal);

        auto observe_res = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/aruco_single/pose", ros::Duration(8.0));
        // ROS_ERROR("Where is bug");
        auto now = ros::Time::now();
        if (observe_res != nullptr && now - observe_res->header.stamp < ros::Duration(2.0)) {
            ROS_INFO("%d, %d", observe_res->header.stamp.nsec, observe_res->header.stamp.sec);
            ROS_INFO("%d, %d", now.nsec, now.sec);
            return true;
        }

        // ??????????????????????????????
        ROS_INFO("Observe action: (%s) failed", params.name.c_str());

        // add this wp to KB
        KnowledgeItem observe_item;
        observe_item.knowledge_type = KnowledgeItem::FACT;
        observe_item.initial_time = ros::Time(0, 0);
        observe_item.attribute_name = "observe";

        vector<KnowledgeItem> to_be_updated = { observe_item };

        KnowledgeUpdateServiceArray srv;
        srv.request.update_type = vector<uint8_t>(to_be_updated.size(), KnowledgeUpdateServiceArray::Request::REMOVE_KNOWLEDGE);
        srv.request.knowledge = move(to_be_updated);

        update_knowledge_array_client_.call(srv);
        return false;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "observe_action_interface");
    rosplane::RPObserveAction oa;
    oa.runActionInterface();
    return 0;
}


