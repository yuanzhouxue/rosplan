#include <despot/planner.h>
#include <tiago_world.h>
#include <tiago_pick_nav.h>


#define ros_info(str, ...) ROS_INFO("%s:%d: " str, __FILE__, __LINE__, __VA_ARGS__)

using namespace despot;

class MyPlanner : public Planner {
    int argc;
    char** argv;

  public:
    double searching_time = 0.0;
    double updating_time = 0.0;
    MyPlanner() {}

    MyPlanner(int argc, char** argv) : argc(argc), argv(argv) {}

    DSPOMDP *InitializeModel(option::Option *options) override {
        DSPOMDP *model = new TiagoModel();
        return model;
    }

    World *InitializeWorld(std::string &world_type, DSPOMDP *model,
                           option::Option *options) override {
        // Create a custom world as defined and implemented by the user
        World *world = new TiagoWorld(argc, argv);
        // Establish connection with external system
        world->Connect();
        // Initialize the state of the external system
        world->Initialize();
        // Inform despot the type of world
        world_type = "simulator";
        return world;
    }

    void InitializeDefaultParameters() override {
        Globals::config.pruning_constant = 0.01;
    }

    std::string ChooseSolver() { return "DESPOT"; }

    /*Customize your planning pipeline by overloading the following function if
     * necessary*/
    void PlanningLoop(Solver *&solver, World *world, Logger *logger) override {
        for (int i = 0; i < Globals::config.sim_len; i++) {
            bool terminal = RunStep(solver, world, logger);
            if (terminal)
                break;
        }
    }

    /*Customize the inner step of the planning pipeline by overloading the
     * following function if necessary*/
    bool RunStep(Solver *solver, World *world, Logger *logger) override {
        logger->CheckTargetTime();

        double step_start_t = get_time_second();

        double start_t = get_time_second();
        ACT_TYPE action = solver->Search().action;
        double end_t = get_time_second();
        double search_time = (end_t - start_t);
        ROS_INFO_STREAM("[Custom RunStep] Time spent in " << typeid(*solver).name()
             << "::Search(): " << search_time);
            
        searching_time += search_time;

        OBS_TYPE obs;
        start_t = get_time_second();
        bool terminal = world->ExecuteAction(action, obs);
        end_t = get_time_second();
        double execute_time = (end_t - start_t);
        ROS_INFO_STREAM("[Custom RunStep] Time spent in ExecuteAction(): "
             << execute_time);

        start_t = get_time_second();
        solver->BeliefUpdate(action, obs);
        end_t = get_time_second();
        double update_time = (end_t - start_t);
        ROS_INFO_STREAM("[Custom RunStep] Time spent in Update(): " << update_time);

        updating_time += update_time;

        return logger->SummarizeStep(step_++, round_, terminal, action, obs,
                                     step_start_t);
    }

    double getSearchingTime() { return searching_time; }
    double getUpdatingTime() { return updating_time; }
};

int main(int argc, char *argv[]) {
    Planner* planner = new MyPlanner(argc, argv);
    double startTime = get_time_second();
    auto res = planner->RunPlanning(argc, argv);
    double endTime = get_time_second();

    MyPlanner* myPlanner = dynamic_cast<MyPlanner*>(planner);
    ROS_INFO_STREAM("[Custom RunPlanning] Time spent in RunPlanning(): " << endTime - startTime);
    ROS_INFO("Searching time: %f", myPlanner->getSearchingTime());
    ROS_INFO("Updating time: %f", myPlanner->getUpdatingTime());
    ROS_INFO("Total planning time: %f", myPlanner->getSearchingTime() + myPlanner->getUpdatingTime());
    ROS_INFO("Total running time: %f", endTime - startTime);
    return res;
}
