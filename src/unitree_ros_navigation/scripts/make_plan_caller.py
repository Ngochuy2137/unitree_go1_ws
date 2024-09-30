#! /usr/bin/env python3

import rospy
from nav_msgs.srv import GetPlan, GetPlanRequest
import sys

rospy.init_node('make_plan_caller_node')
rospy.wait_for_service('/move_base/make_plan')
make_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

req = GetPlanRequest()
req.start.header.frame_id = 'map'
req.start.pose.position.x = 0
req.start.pose.position.y = 0
req.start.pose.position.z = 0
req.start.pose.orientation.x = 0
req.start.pose.orientation.y = 0
req.start.pose.orientation.z = 0
req.start.pose.orientation.w = 0

req.goal.header.frame_id = 'map'
req.goal.pose.position.x = 2
req.goal.pose.position.y = 6
req.goal.pose.position.z = 0
req.goal.pose.orientation.x = 0
req.goal.pose.orientation.y = 0
req.goal.pose.orientation.z = 0
req.goal.pose.orientation.w = 0

result = make_plan_service(req)
print(result)
