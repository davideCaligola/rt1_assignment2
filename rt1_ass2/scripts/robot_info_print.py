#!/usr/bin/env python3

from __future__ import print_function
import rospy
from rt1_ass2.msg import Robot_info
from rt1_ass2.srv import GoalPosition
import math

print_timer = None
robot_current_goal_pos_client = None
dist_to_goal = 0.0,
vel_cumulative = 0.0
number_of_samples = 0

def robot_info_callback(robot_info: Robot_info):
    global dist_to_goal, vel_cumulative, number_of_samples
    global robot_current_goal_pos_client

    # get goal position
    try:
        current_goal_pos = robot_current_goal_pos_client()
    except rospy.ServiceException as e:
        rospy.logwarn("Service call to robot_current_goal_pos failed %s" %e)
    
    # calculate goal distance and cumulative linear velocity
    if current_goal_pos.valid:

        # calculate distance to goal
        dist_to_goal = math.sqrt((current_goal_pos.x - robot_info.x)**2 + (current_goal_pos.y - robot_info.y)**2)        

        # update cumulative velocity
        vel_cumulative += math.sqrt((robot_info.vel_x)**2 + (robot_info.vel_y)**2)
        number_of_samples += 1

    else:
        dist_to_goal = 0.0
        vel_cumulative = 0.0
        number_of_samples = 0

    
def print_callback(event):
    global dist_to_goal, vel_cumulative, number_of_samples
    
    rospy.loginfo("============================")
    rospy.loginfo("distance to goal: %1.3f", dist_to_goal)
    rospy.loginfo("average linear velocity: %1.3f", 0.0 if number_of_samples == 0 else (vel_cumulative/number_of_samples))

def remove_timer():
    global print_timer
    
    # stop timer
    print_timer.shutdown()

def main():
    # global variable initialization
    global print_timer
    global dist_to_goal, vel_cumulative
    global robot_current_goal_pos_client

    dist_to_goal = 0
    vel_cumulative = 0

    # node initialization
    rospy.init_node("robot_stat_print")

    # get printing frequency
    print_freq = float(rospy.get_param("/print_freq"))
    rospy.loginfo("print_freq %1.3f", print_freq)

    # create subscriber for /robot_info
    robot_info_sub = rospy.Subscriber("/robot_info_odom", Robot_info, callback=robot_info_callback)
    
    rospy.loginfo("Waiting for robot_current_goal_pos service...")
    rospy.wait_for_service("robot_current_goal_pos")
    rospy.loginfo("Service robot_current_goal_pos available")

    # client for getting current goal position
    robot_current_goal_pos_client = rospy.ServiceProxy("robot_current_goal_pos", GoalPosition)

    # define timer for printing robot stats
    print_timer = rospy.Timer(rospy.Duration(1.0/print_freq), callback=print_callback)

    # hook for shutdown
    rospy.on_shutdown(remove_timer)

    rospy.loginfo("Node robot_stat_print initialized")

    rospy.spin()

if __name__ == "__main__":
    main()
