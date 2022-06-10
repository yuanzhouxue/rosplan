#include <despot/interface/world.h>
#include <ros/ros.h>
using namespace despot;

class TiagoWorld: public World {
    ros::NodeHandlePtr nh;
    ros::ServiceClient pick_client, place_client, goto_client;
    ros::ServiceClient inhand_client;
    int argc;
    char** argv;
    State* state_;
public:
    TiagoWorld(int argc, char** argv) : argc(argc), argv(argv) {};

    bool Connect() override;

    //Initialize or reset the environment (for simulators or POMDP world only), return the start state of the system if applicable
    State* Initialize() override;

    //Get the state of the system (only applicable for simulators or POMDP world)
    State* GetCurrentState() const override;

    //Send action to be executed by the system, receive observations terminal signals from the system
    bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) override;
};
