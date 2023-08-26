#!/usr/bin/env python3
"""
In this node, a rough textual interface has been developed to interact with the action
server provided by the given package
`assignment_2_2002 <https://github.com/davideCaligola/rt1_assignment2/tree/main/assignment\_2\_2022>`_
through a respective action client. It allows to

- set a goal to reach for the robot
- cancel the current goal
- get the information about the number of goals reached or canceled
- quit the simulation.

Furthermore, this node publishes the topic ``/robot_info_odom`` containing
the planar position and linear velocity of the robot extracted by the topic ``/odom``
For convenience, it also provides the service ``robot_current_goal_pos`` which provides
the planar position coordinates (x,y) of the current goal.
"""

from __future__ import print_function
import rospy
import assignment_2_2022.msg
from nav_msgs.msg import Odometry
from rt1_ass2.msg import Robot_info
from rt1_ass2.srv import *
import threading
import actionlib
from actionlib_msgs.msg import GoalStatusArray
import math

robot_info_odom_pub = None
goal_action_client = None
goal_status_sub = None
goal_position = None
goal_pos_srv = None
goal_result_counts_client = None
state = 0
# states:
# 0: requesting command
# 1: send goal to server
# 2: cancel goal
# 3: get goal result counting
# 4: terminate node
# 5: waiting for goal active
# 6: waiting for goal canceled


def prompt_get_command():
    """
    Prints on console the menu for choosing an action to perform

    :return: None
    """
    print("\n===============================")
    print("Choose your action:\n" +
          "g: set a new goal\n" +
          "c: cancel current goal\n" +
          "s: get counter on how many goals have been reached/canceled" +
          "q: quit (it takes time to shutdown everything, so please wait a little...)")
    return input("Command: ")


def saturate_input(x:float, limit:float):
    """
    Symmetric saturation of the number x to limit, so that \|x\| <= limit.

    :param x: number to saturate to limit
    :type x: float
    :param limit: symmetric saturation limit
    :type limit: float
    :return: x if \|x\| < limit; sign(x)*limit if \|x\| >= limit
    :rtype: float
    """
    if (abs(x) > limit):
        return  math.copysign(limit, x)
    else:
        return x


def command_handling():
    """
    Handles the input command from command console. If command is 

    - ``g``: it requests and aquires goal coordinates and moves state machine to state 1 (send goal to service)
    - ``c``: if there is a current goal defined, moves state machine to state 2 (cancel goal)
    - ``s``: moves state machine to state 3 (get goal result counts)
    - ``q``: moves state machine to state 4 (terminate node)

    :return: None
    """
    global state
    global goal_action_client
    global goal_position

    if goal_position == None:
        goal_position = assignment_2_2022.msg.PlanningActionGoal().goal
    
    # symmetric coordinate limits in absolute value 
    x_lim = 8
    y_lim = 8

    cmd = ""
    if cmd == "":       # print command request
            cmd = prompt_get_command()

    if cmd == 'g':      # set a new goal
        cmd = ""
        print("\nPlease enter the goal to reach (the values are automatically limited in the range [-8,8])")
        try:
            goal_position.target_pose.pose.position.x = saturate_input(float(input("x: ")), x_lim)
            goal_position.target_pose.pose.position.y = saturate_input(float(input("y: ")), y_lim)
        except Exception as e:
            print("Something went wrong entering the goal coordinates:\n%s" %e)
            cmd = ""
            return
        
        print(f"Sending goal: ("+
                "{:.2f},".format(goal_position.target_pose.pose.position.x)+
                "{:.2f})".format(goal_position.target_pose.pose.position.y))
        
        state = 1 # send goal
        return
    
    elif cmd == 'c':    # cancel current goal
        cmd = ""
        # check if there is a goal to preempt
        goal_state = goal_action_client.get_state()
        if (goal_action_client.get_state() == actionlib.GoalStatus.LOST or
            goal_action_client.get_state() == actionlib.GoalStatus.SUCCEEDED):
            print("\nThere is not any goal to delete")
        else:
            print("\nRemoving current goal...")
            state = 2 # cancel goal
    
    elif cmd == 's':    # get goal result counts
        state = 3

    elif cmd == 'q':    # terminate node
        state = 4

    else:
        # reset command
        cmd = ""
        print("\nWrong command!")


def send_goal_request():
    """
    - Sends the new goal coordinates to the service ``reaching_goal``
    - Subscribes to the topic ``/reaching_goal/status``
    - Moves state machine to state 5 (waiting for goal to be active)

    :return: None
    """
    global goal_action_client
    global state
    global goal_status_sub
    global goal_pos_srv
    
    # create subscriber for goal result
    goal_status_sub = rospy.Subscriber("/reaching_goal/status", GoalStatusArray, goal_status_callback)

    # set current goal position for server
    goal_pos_srv = GoalPositionResponse()
    goal_pos_srv.valid = True
    goal_pos_srv.x = goal_position.target_pose.pose.position.x
    goal_pos_srv.y = goal_position.target_pose.pose.position.y
    
    print("send goal")
    # send goal request
    goal_action_client.send_goal(goal_position)

    state = 5 # wait for goal to be active

def reset_goal_position():
    """
    Sets to None global variables ``goal_position`` and ``goal_pos_srv``

    :return: None
    """
    global goal_position
    global goal_pos_srv

    goal_position = None
    goal_pos_srv = None

def cancel_goal():
    """
    - subscribes to topic ``/reaching_goal/status``
    - sends the request to cancel the goal to the service ``/reaching_goal``
    - resets internal goal position calling :func:`reset_goal_position`
    - moves state machine to state 6 (waiting for goal to be preempted)

    :return: None
    """
    global goal_action_client
    global goal_status_sub
    global goal_position
    global state
    
    # create subscriber for goal result
    goal_status_sub = rospy.Subscriber("/reaching_goal/status", GoalStatusArray, goal_status_callback)

    # cancel current goal
    goal_action_client.cancel_goal()

    # reset goal position
    reset_goal_position()

    state = 6 # wait for goal to be preempted

def get_goal_result_counts():
    """
    - requests goal result counters to the service ``goal_result_counter`` (timeout 10.0s)
    - prints on console the information about goals reached and goals canceld
    - moves state machine to state 0 (waiting for input command)

    :return: None
    """
    global state, goal_result_counts_client
    timeout = 10.0    # (seconds) timeout for connecting to service goal_result_counter
    timeout_goal_result_counter = rospy.Duration(timeout)

    # check for service
    print("\nWaiting for goal_result_counter service ({:.1f}s)...".format(timeout))
    try:
        rospy.wait_for_service("goal_result_counter", timeout=timeout_goal_result_counter)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn("Error while waiting for the service goal_result_counter.\n%s" %e)
        state = 0
        return
    
    # create client, if not defined yet
    if goal_result_counts_client == None:
        # create client for getting goal result counts
        goal_result_counts_client = rospy.ServiceProxy("goal_result_counter", GoalResultCounter)
    
    # get goal result counters
    try:
        goal_result_counts = goal_result_counts_client()
        
        print("Goals reached: {:d}".format(goal_result_counts.reached))
        print("Goals canceled: {:d}".format(goal_result_counts.canceled))
    
    except rospy.ServiceException as e:
        rospy.logwarn("Service call to goal_result_counter failed %s" %e)

    state = 0
    

def terminate_node():
    """
    Sends the shutdonw signal to the node

    :return: None
    """
    rospy.signal_shutdown("\nShutting down node on user request...")


def goal_status_callback(goal_status: GoalStatusArray):
    """
    Callback function for the topic ``/reaching_goal/status``.

    - if goal status is ``ACTIVE`` and state machine is in state 5 (waiting for goal active)
      the robot is moving toward the target. It moves the state machine to state 0
      (waiting for input command)
    - if goal status is ``PREEMPTED`` and state machine is in state 6
      (waiting for goal to be preempted), the current goal has been removed. It moves the state
      machine to state 0 (waiting for input command)
    - if goal status is ``SUCCEEDED`` and state machine is in state 0 (waiting for input command)
      the current goal has been reached. It removes the subscription to topic ``/reaching_goal/status``
      and resets internal goal calling function :func:`reset_goal_position`

    :param goal_status: new goal status provided by topic ``/reaching_goal/status``
    :type goal_status: GoalStatusArray
    :return: None
    """
    global state
    global goal_status_sub

    # check that status list is not empty
    if goal_status.status_list:
        
        # waiting for activating the goal do to a new goal request
        if (goal_status.status_list[0].status == actionlib.GoalStatus.ACTIVE and state == 5):
            print("Moving toward the goal...")
            state = 0
            return

        # waiting for removing the goal due to a cancel goal request
        elif (goal_status.status_list[0].status == actionlib.GoalStatus.PREEMPTED and state == 6):
            goal_status_sub.unregister()
            print("Goal removed")
            state = 0
            return
        
        # reset goal position once current goal is reached
        elif (goal_status.status_list[0].status == actionlib.GoalStatus.SUCCEEDED and state == 0):
            goal_status_sub.unregister()
            
            reset_goal_position()


def goal_management():
    """
    - creates a client for the service ``/reaching_goal``,
    - defines the state machine for handling the different phases of the process:

      * state 0: waiting for input command
      * state 1: send goal request to service ``/reacing_goal``
      * state 2: cancel current goal
      * state 3: get goal result counters (reached and canceled)
      * state 4: terminate the node
      * state 5: waiting for goal to become active
      * state 6: waiting for current goal to be preempted

    :return: None
    """
    global state
    global goal_position
    global goal_action_client

    # create simple action client
    goal_action_client = actionlib.SimpleActionClient("/reaching_goal", assignment_2_2022.msg.PlanningAction)

    # wait for action server
    rospy.loginfo("waiting for server /reaching_goal...")
    goal_action_client.wait_for_server()
    rospy.loginfo("Server /reaching_goal available")

    # manage goal
    while (not rospy.is_shutdown()):
        if state == 0:
            command_handling()
        
        elif state == 1:
            send_goal_request()

        elif state == 2:
            cancel_goal()
        
        elif state == 3:
            get_goal_result_counts()

        elif state == 4:
            terminate_node()

        elif state == 5:
            # waiting for goal active
            pass

        elif state == 6:
            # waiting for goal preempted
            pass


# odometry subscriber callback
# publishes the robot info everytime
# it gets a new odometry update
def odom_callback(odom: Odometry):
    """
    Callback function for topic ``/odom``.

    - acquires the planar position and velocity form the topic ``/odom``
    - publishes those data on the topic ``/robot_info_odom``

    :param odom: argument of the topic ``/odom``
    :type odom: Odometry
    :return: None
    """
    global robot_info_odom_pub

    # define message to publish
    robot_info = Robot_info()

    # set robot info values
    robot_info.x = odom.pose.pose.position.x
    robot_info.y = odom.pose.pose.position.y
    robot_info.vel_x = odom.twist.twist.linear.x
    robot_info.vel_y = odom.twist.twist.linear.y

    # publish robot info
    robot_info_odom_pub.publish(robot_info)


def robot_current_goal_pos_handler(req: GoalPositionRequest):
    """
    Handle for service ``robot_current_goal_pos``. If a goal is defined, it returns the current goal position;
    if it is not defined, it returns an empty object.

    :param req: not used. Default argument for service request
    :type req: GoalPositionRequest
    :return: current goal position
    :rtype: :ref:`GoalPositionResponse <goalPosition_ref>`
    """
    global goal_pos_srv

    # if there is not any current goal selected,
    # return an empty dictionary
    return goal_pos_srv if goal_pos_srv else {}


# node main function
def main():
    """
    Main function of the node
      - initialize the node
      - creates publisher for topic ``/robot_info_odom``
      - creates subscriber for topic ``/odom`` of given package
        `assignment_2_2002 <https://github.com/davideCaligola/rt1_assignment2/tree/main/assignment\_2\_2022>`_
      - creates server for service ``robot_current_goal_pos``
      - launch function :func:`goal_management` for handling input command in a new thread

    :return: None
    """
    global goal_management_worker
    global robot_info_odom_pub

    # initialize node
    rospy.init_node("robot_command")
    
    # creates publisher for Robot info
    robot_info_odom_pub = rospy.Publisher("/robot_info_odom", Robot_info, queue_size=10)

    # creates subscriber for /odom
    odom_sub = rospy.Subscriber("/odom", Odometry, callback=odom_callback)

    # creates server for providing current goal position
    robot_current_goal_pos_srv = rospy.Service("robot_current_goal_pos", GoalPosition, robot_current_goal_pos_handler)

    rospy.loginfo("Node robot_command has been initialized")

    # creates thread for goal management
    goal_management_worker = threading.Thread(target=goal_management)
    goal_management_worker.start()

    rospy.spin()

if __name__ == "__main__":
    main()
    