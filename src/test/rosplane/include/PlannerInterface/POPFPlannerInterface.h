#include "PlannerInterface/PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

#ifndef ROSPLANE_POPF_planner_interface
#define ROSPLANE_POPF_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace rosplane {

	class POPFPlannerInterface: public PlannerInterface
	{
	private:

		/* runs external commands */
		std::string runCommand(std::string cmd);
        bool runPlanningServerPDDL(std::string domainPath, std::string problemPath, std::string dataPath, std::string plannerCommand, bool useProblemTopic);

	protected:

		bool runPlanner();


	public:

		POPFPlannerInterface(ros::NodeHandle& nh);
		virtual ~POPFPlannerInterface();
	};

} // close namespace

#endif
