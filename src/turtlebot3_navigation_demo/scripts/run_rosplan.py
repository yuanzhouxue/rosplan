#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from std_srvs.srv import Empty
import time

if __name__ == "__main__":
    rospy.wait_for_service("/rosplan_problem_interface/problem_generation_server")
    rospy.wait_for_service("/rosplan_planner_interface/planning_server")
    rospy.wait_for_service("/rosplan_parsing_interface/parse_plan")
    rospy.wait_for_service("/rosplan_plan_dispatcher/dispatch_plan")

    problem_gen_client = rospy.ServiceProxy("/rosplan_problem_interface/problem_generation_server", Empty)
    planning_client = rospy.ServiceProxy("/rosplan_planner_interface/planning_server", Empty)
    parsing_client = rospy.ServiceProxy("/rosplan_parsing_interface/parse_plan", Empty)
    dispatch_client = rospy.ServiceProxy("/rosplan_plan_dispatcher/dispatch_plan", Empty)


    start_time = time.time()
    problem_gen_client.call()
    planning_client.call()
    parsing_client.call()
    
    planning_end_time = time.time()
    dispatch_client.call()
    task_end_time = time.time()

    rospy.loginfo(f"Total planning time: {planning_end_time - start_time}s.")
    rospy.loginfo(f"Total running time: {task_end_time - start_time}s.")