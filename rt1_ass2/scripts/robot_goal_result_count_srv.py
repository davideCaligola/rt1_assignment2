#!/usr/bin/env python3

import rospy
from rt1_ass2.srv import *
from actionlib import GoalStatus
from assignment_2_2022.msg import *

reached = 0
canceled = 0

def goal_status_callback(goal_status: PlanningActionResult):
    global reached
    global canceled

    # goal reached
    if (goal_status.status.status == GoalStatus.SUCCEEDED):
        print("goal reached")
        reached += 1
        return
    
    # goal canceled
    elif (goal_status.status.status == GoalStatus.PREEMPTED or
          goal_status.status.status == GoalStatus.ABORTED):
        print("goal canceled")
        canceled += 1
        return
            

def robot_goal_count_handle(req: GoalResultCounterRequest):
    global reached, canceled
    
    counter = GoalResultCounterResponse()
    counter.reached = reached
    counter.canceled = canceled

    return counter
    

def main():
    # instantiate node
    rospy.init_node("robot_goal_result_count_srv")

    # subscription to goal status changes
    goal_status_sub = rospy.Subscriber("/reaching_goal/result", PlanningActionResult, goal_status_callback)

    # create service for goal result cunter
    goal_counter_result_srv = rospy.Service("goal_result_counter", GoalResultCounter, robot_goal_count_handle)

    rospy.loginfo("Node robot_goal_result_count_srv initialized")
    
    rospy.spin()

if __name__ == "__main__":
    main()
