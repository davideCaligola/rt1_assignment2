#!/usr/bin/env python3

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
    print("\n===============================")
    print("Choose your action:\n" +
          "g: set a new goal\n" +
          "c: cancel current goal\n" +
          "s: get counter on how many goals have been reached/canceled" +
          "q: quit (it takes time to shutdown everything, so please wait a little...)")
    return input("Command: ")


def saturate_input(x, limit):
    if (abs(x) > limit):
        return  math.copysign(limit, x)
    else:
        return x


def command_handling():
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
    global goal_position
    global goal_pos_srv

    goal_position = None
    goal_pos_srv = None

def cancel_goal():
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
    rospy.signal_shutdown("\nShutting down node on user request...")


def goal_status_callback(goal_status: GoalStatusArray):
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
    global goal_pos_srv

    # if there is not any current goal selected,
    # return an empty dictionary
    return goal_pos_srv if goal_pos_srv else {}


# node main function
def main():
    global goal_management_worker
    global robot_info_odom_pub

    # initialize node
    rospy.init_node("robot_command")
    
    # create publisher for Robot info
    robot_info_odom_pub = rospy.Publisher("/robot_info_odom", Robot_info, queue_size=10)

    # create subscriber for /odom
    odom_sub = rospy.Subscriber("/odom", Odometry, callback=odom_callback)

    # create server for providing current goal position
    robot_current_goal_pos_srv = rospy.Service("robot_current_goal_pos", GoalPosition, robot_current_goal_pos_handler)

    rospy.loginfo("Node robot_command has been initialized")

    # create thread for goal management
    goal_management_worker = threading.Thread(target=goal_management)
    goal_management_worker.start()

    rospy.spin()

if __name__ == "__main__":
    main()
    