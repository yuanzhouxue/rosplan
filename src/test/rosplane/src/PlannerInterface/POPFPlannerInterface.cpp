#include "PlannerInterface/POPFPlannerInterface.h"

namespace rosplane {

    /*-------------*/
    /* constructor */
    /*-------------*/

    POPFPlannerInterface::POPFPlannerInterface(ros::NodeHandle& nh) {
        node_handle = &nh;

        plan_server = new actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>((*node_handle), "start_planning", boost::bind(&PlannerInterface::runPlanningServerAction, this, _1), false);

        // publishing raw planner output
        std::string plannerTopic = "planner_output";
        node_handle->getParam("planner_topic", plannerTopic);
        plan_publisher = node_handle->advertise<std_msgs::String>(plannerTopic, 1, true);
        ROS_INFO("Advertise planning server");

        // start planning action server
        plan_server->start();
    }

    POPFPlannerInterface::~POPFPlannerInterface() {
        delete plan_server;
    }

    /**
     * Runs external commands
     */
    std::string POPFPlannerInterface::runCommand(std::string cmd) {
        std::string data;
        FILE* stream;
        char buffer[1000];
        stream = popen(cmd.c_str(), "r");
        while (fgets(buffer, 1000, stream) != NULL)
            data.append(buffer);
        pclose(stream);
        return data;
    }

    /*------------------*/
    /* Plan and process */
    /*------------------*/

    /**
     * passes the problem to the Planner; the plan to post-processing.
     */
    bool POPFPlannerInterface::runPlanner() {

        // save problem to file for POPF
        if (use_problem_topic && problem_instance_received) {
            ROS_INFO("KCL: (%s) (%s) Writing problem to file.", ros::this_node::getName().c_str(), problem_name.c_str());
            std::ofstream dest;
            dest.open((problem_path).c_str());
            dest << problem_instance;
            dest.close();
        }

        // prepare the planner command line
        std::string str = planner_command;
        std::size_t dit = str.find("DOMAIN");
        if (dit != std::string::npos) str.replace(dit, 6, domain_path);
        std::size_t pit = str.find("PROBLEM");
        if (pit != std::string::npos) str.replace(pit, 7, problem_path);
        std::string commandString = str + " > " + data_path + "plan.pddl";

        // call the planer
        ROS_INFO("KCL: (%s) (%s) Running: %s", ros::this_node::getName().c_str(), problem_name.c_str(), commandString.c_str());
        std::string plan = runCommand(commandString.c_str());
        ROS_INFO("KCL: (%s) (%s) Planning complete", ros::this_node::getName().c_str(), problem_name.c_str());

        // check the planner solved the problem
        std::ifstream planfile;
        planfile.open((data_path + "plan.pddl").c_str());
        std::string line;
        std::stringstream ss;

        int curr, next;
        bool solved = false;
        double planDuration;

        while (std::getline(planfile, line)) {

            if (line.find("; Plan found", 0) != std::string::npos || line.find(";;;; Solution Found", 0) != std::string::npos) {
                solved = true;
            } else if (line.find("; Time", 0) == std::string::npos) {
                // consume useless lines
            } else {
                // read a plan (might not be the last plan)
                planDuration = 0;
                ss.str("");
                while (std::getline(planfile, line)) {
                    if (line.length() < 2)
                        break;
                    ss << line << std::endl;
                }
                planner_output = ss.str();
            }
        }
        planfile.close();

        if (!solved) ROS_INFO("KCL: (%s) (%s) Plan was unsolvable.", ros::this_node::getName().c_str(), problem_name.c_str());
        else ROS_INFO("KCL: (%s) (%s) Plan was solved.", ros::this_node::getName().c_str(), problem_name.c_str());

        return solved;
    }

} // close namespace

    /*-------------*/
    /* Main method */
    /*-------------*/

int main(int argc, char** argv) {

    srand(static_cast <unsigned> (time(0)));

    ros::init(argc, argv, "rosplane_planner_interface");
    ros::NodeHandle nh("~");

    rosplane::POPFPlannerInterface pi(nh);

    // subscribe to problem instance
    std::string problemTopic = "problem_instance";
    nh.getParam("problem_topic", problemTopic);
    ros::Subscriber problem_sub = nh.subscribe(problemTopic, 1, &rosplane::PlannerInterface::problemCallback, dynamic_cast<rosplane::PlannerInterface*>(&pi));

    // start the planning services
    ros::ServiceServer service1 = nh.advertiseService("planning_server", &rosplane::PlannerInterface::runPlanningServerDefault, dynamic_cast<rosplane::PlannerInterface*>(&pi));
    ros::ServiceServer service2 = nh.advertiseService("planning_server_params", &rosplane::PlannerInterface::runPlanningServerParams, dynamic_cast<rosplane::PlannerInterface*>(&pi));
    ros::ServiceServer service3 = nh.advertiseService("planning_pddl", &rosplane::PlannerInterface::runPlanningServerPDDLDefault, dynamic_cast<rosplane::PlannerInterface*>(&pi));
    ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}
