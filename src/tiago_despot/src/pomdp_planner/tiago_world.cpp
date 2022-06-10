
#include <ros/ros.h>
#include <tiago_despot/ActionSrv.h>
#include <tiago_despot/SensorSrv.h>
// for tests
#include <iostream>

#include "tiago_pick_nav.h"
#include "tiago_world.h"

using namespace despot;

bool TiagoWorld::Connect() {

    // initialize ROS node
    ros::init(argc, argv, "tiago_despot");
    nh = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    // get laser's noise sigma
    string pick_node_name, place_node_name, goto_node_name;
    string inhand_node_name;

    nh->getParam("pick_node_name", pick_node_name);
    nh->getParam("place_node_name", place_node_name);
    nh->getParam("goto_node_name", goto_node_name);
    nh->getParam("inhand_node_name", inhand_node_name);

    string pick_service_name = "/" + pick_node_name + "/my_pick_service";
    string place_service_name = "/" + place_node_name + "/my_place_service";
    string goto_service_name = "/" + goto_node_name + "/my_goto_service";
    string inhand_service_name = "/" + inhand_node_name + "/my_inhand_sensor";

    // wait for laser tag controller service to show up (blocking call)
    ros::service::waitForService(pick_service_name, -1);
    ros::service::waitForService(place_service_name, -1);
    ros::service::waitForService(goto_service_name, -1);
    ros::service::waitForService(inhand_service_name, -1);

    // setup service client
    pick_client = nh->serviceClient<tiago_despot::ActionSrv>(pick_service_name);
    place_client =
        nh->serviceClient<tiago_despot::ActionSrv>(place_service_name);
    goto_client = nh->serviceClient<tiago_despot::ActionSrv>(goto_service_name);
    inhand_client = nh->serviceClient<tiago_despot::SensorSrv>(inhand_service_name);
}

// Initialize or reset the environment (for simulators or POMDP world only),
// return the start state of the system if applicable
State *TiagoWorld::Initialize() {
    int initial_state = 0;
    initial_state |= 1 << TiagoModel::indexOf(TiagoModel::propositions_,
                                              {"robot_at", "wp2"});
    initial_state |=
        1 << TiagoModel::indexOf(TiagoModel::propositions_, {"empty_hand"});
    initial_state |=
        1 << TiagoModel::indexOf(TiagoModel::propositions_, {"guest_not_near"});
    initial_state |= 1 << TiagoModel::indexOf(TiagoModel::propositions_,
                                              {"near", "wp0", "table0"});
    initial_state |= 1 << TiagoModel::indexOf(TiagoModel::propositions_,
                                              {"near", "wp1", "table1"});
    initial_state |= 1 << TiagoModel::indexOf(TiagoModel::propositions_,
                                              {"on_table", "table0"});
    state_ = new State(initial_state, 1.0);
    return state_;
}

// Get the state of the system (only applicable for simulators or POMDP world)
State *TiagoWorld::GetCurrentState() const { return state_; }

// Send action to be executed by the system, receive observations terminal
// signals from the system
bool TiagoWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE &obs) {
    tiago_despot::ActionSrv srv;
    // TiagoModel::actions_
    const auto &actionName = TiagoModel::actions_[action][0];
    if (actionName == "pick") {
        srv.request.action.name = "pick";
        const auto &paraTable = TiagoModel::actions_[action][1];
        const auto &paraWaypoint = TiagoModel::actions_[action][2];

        auto cond1 = 1 << TiagoModel::indexOf(TiagoModel::propositions_, {"robot_at", paraWaypoint});
        auto cond2 =
            1 << TiagoModel::indexOf(TiagoModel::propositions_, {"near", paraWaypoint, paraTable});
        auto cond3 = 1 << TiagoModel::indexOf(TiagoModel::propositions_, {"on_table", paraTable});
        auto cond4 = 1 << TiagoModel::indexOf(TiagoModel::propositions_, {"empty_hand"});
        auto cond = cond1 | cond2 | cond3 | cond4;
        if ((state_->state_id & cond) != cond) {
            obs = 0;
        } else {
            pick_client.call(srv);
            tiago_despot::SensorSrv sensorSev;
            inhand_client.call(sensorSev);

            if (sensorSev.response.sensor_success) {
                obs = 1 << TiagoModel::indexOf(TiagoModel::obs_, {"in_hand"});;
                auto eff1 = TiagoModel::indexOf(TiagoModel::propositions_,
                                                {"in_hand"});
                auto eff2 = TiagoModel::indexOf(TiagoModel::propositions_,
                                                {"empty_hand"});
                auto eff3 = TiagoModel::indexOf(TiagoModel::propositions_,
                                                {"on_table", paraTable});
                state_->state_id |= 1 << eff1;
                state_->state_id &= ~((1 << eff2) | (1 << eff3));
            } else {
                obs = (1 << TiagoModel::indexOf(TiagoModel::obs_, {"empty_hand"})) | (1 << TiagoModel::indexOf(TiagoModel::obs_, {"on_table", paraTable}));
            }
        }

    } else if (actionName == "place") {
        srv.request.action.name = "place";
        const auto &paraTable = TiagoModel::actions_[action][1];
        const auto &paraWaypoint = TiagoModel::actions_[action][2];
        auto obs1 = 1 << TiagoModel::indexOf(TiagoModel::obs_, {"empty_hand"});
        auto obs2 =
            1 << TiagoModel::indexOf(TiagoModel::obs_, {"on_table", paraTable});

        auto cond1 = 1 << TiagoModel::indexOf(TiagoModel::propositions_, {"robot_at", paraWaypoint});
        auto cond2 =
            1 << TiagoModel::indexOf(TiagoModel::propositions_, {"near", paraWaypoint, paraTable});
        auto cond3 = 1 << TiagoModel::indexOf(TiagoModel::propositions_, {"in_hand"});
        auto cond4 = 1 << TiagoModel::indexOf(TiagoModel::propositions_, {"guest_not_near"});
        auto cond = cond1 | cond2 | cond3 | cond4;
        if ((state_->state_id & cond) != cond) {
            obs = 0;
        } else {
            if (place_client.call(srv)) {
                if (srv.response.action_success) {
                    obs = obs1 | obs2;
                    auto eff1 = TiagoModel::indexOf(TiagoModel::propositions_,
                                                    {"in_hand"});
                    auto eff2 = TiagoModel::indexOf(TiagoModel::propositions_,
                                                    {"empty_hand"});
                    auto eff3 = TiagoModel::indexOf(TiagoModel::propositions_,
                                                    {"on_table", paraTable});
                    state_->state_id |= (1 << eff2) | (1 << eff3);
                    state_->state_id &= ~(1 << eff1);
                } else {
                    obs = 1 << TiagoModel::indexOf(TiagoModel::obs_, {"in_hand"});
                }
            }
        }
    } else if (actionName == "goto") {
        srv.request.action.name = "goto_waypoint";
        const auto &paraFrom = TiagoModel::actions_[action][1];
        const auto &paraTo = TiagoModel::actions_[action][2];
        diagnostic_msgs::KeyValue kv;
        kv.key = "to";
        kv.value = paraTo;
        srv.request.action.parameters.push_back(kv);

        auto cond1 = 1 << TiagoModel::indexOf(TiagoModel::propositions_,
                                              {"robot_at", paraFrom});
        auto obs1 = 1 << TiagoModel::indexOf(TiagoModel::obs_, {"in_hand"});
        if (cond1 != (state_->state_id & cond1)) {
            obs = 0;
        } else {
            if (goto_client.call(srv)) {
                if (srv.response.action_success) {
                    obs = 1 << TiagoModel::indexOf(TiagoModel::obs_,
                                                   {"robot_at", paraTo});
                    state_->state_id |=
                        1 << TiagoModel::indexOf(TiagoModel::propositions_,
                                                 {"robot_at", paraTo});
                    state_->state_id &=
                        ~(1 << TiagoModel::indexOf(TiagoModel::propositions_,
                                                   {"robot_at", paraFrom}));
                } else {
                    obs = 1 << TiagoModel::indexOf(TiagoModel::obs_,
                                                   {"robot_at", paraFrom});
                }
            }
        }
    } else {
        ROS_ERROR("Unknown action: %s", actionName.c_str());
    }
    auto goal1 = 1 << TiagoModel::indexOf(TiagoModel::propositions_,
                                          {"on_table", "table1"});
    auto goal2 = 1 << TiagoModel::indexOf(TiagoModel::propositions_,
                                          {"robot_at", "wp2"});
    auto goal = goal1 | goal2;
    // auto goal = 1 << indexOf(propositions_, {"in_hand"});
    if ((state_->state_id & goal) == goal) {
        return true;
    }
    return false;
}
