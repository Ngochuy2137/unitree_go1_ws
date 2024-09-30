#! /usr/bin/env python3

import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

goal_list = [[0.18, 1.16, 0.0, 0.0, 0.0, 0.0, 1.0],
             [0.0, -1.9, 0.0, 0.0, 0.0, 0.0, 1.0],
             [0.7, -4.3, 0.0, 0.0, 0.0, 0.0, 1.0]]

def feedback_callback(msg):
    """
    definition of the feedback callback. This will be called when feedback
    is received from the action server
    it just prints a message indicating a new message has been received
    """
    rospy.loginfo("[Feedback] Going to Goal Pose...")

# initializes the action client node
rospy.init_node('move_base_action_client')
# create the connection to the action server
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
# waits until the action server is up and running
client.wait_for_server()

while not rospy.is_shutdown():
    i = 1
    for goal in goal_list:
        # creates a goal to send to the action server
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.pose.position.x = goal[0]
        goal_msg.target_pose.pose.position.y = goal[1]
        goal_msg.target_pose.pose.position.z = goal[2]
        goal_msg.target_pose.pose.orientation.x = goal[3]
        goal_msg.target_pose.pose.orientation.y = goal[4]
        goal_msg.target_pose.pose.orientation.z = goal[5]
        goal_msg.target_pose.pose.orientation.w = goal[6]

        rospy.loginfo("Goal Target " + str(i))

        # sends the goal to the action server, specifying which feedback function
        # to call when feedback received
        client.send_goal(goal=goal_msg, feedback_cb=feedback_callback)
        client.wait_for_result()

        rospy.loginfo("[Result] State: %d"%(client.get_state()))
        i += 1
