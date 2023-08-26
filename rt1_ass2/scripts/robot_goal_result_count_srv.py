#!/usr/bin/env python3
"""
This node implements the service :ref:`goal_result_counter <goalResultCounter_ref>` which, subscribing to the topic 
``/reaching_goal/result`` from the given package
`assignment_2_2002 <https://github.com/davideCaligola/rt1_assignment2/tree/main/assignment\_2\_2022>`_,
provides the number of goal reached or canceled.
"""

import rospy
from rt1_ass2.srv import *
from actionlib import GoalStatus
from assignment_2_2022.msg import *

reached = 0
canceled = 0

def goal_status_callback(goal_status: PlanningActionResult):
    """
    Callback function for service ``/reaching_goal/result``.
    If status is ``SUCCEEDED``, reached goal counter is increaded by 1
    If status is ``PREEMPTED`` or ``ABORTED``, it increases canceled goal counter by 1

    :param goal_status: returns the goal status from its field status.status
    :type goal_status: PlanningActionResult
    :return: None
    """
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
    """
    Handle for the service ``goal_result_counter``.
    It returns an object containing the number of goal reached and canceled.

    :param req: not used. Default request argument for service ``goal_result_counter``
    :type req: GoalResultCounterRequest
    :return: object containing the number of goal reached and canceled
    :rtype: :ref:`GoalResultCounterResponse <goalResultCounter_ref>`
    """
    global reached, canceled
    
    counter = GoalResultCounterResponse()
    counter.reached = reached
    counter.canceled = canceled

    return counter
    

def main():
    """
    Main function of the node
      - initialize the node
      - subscribes to the topic ``/reaching_goal/result`` to get the new goal status every time it changes
      - creates the service ``goal_result_counter`` to provide the number of reached and canceled goals

    :return: None
    """
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
