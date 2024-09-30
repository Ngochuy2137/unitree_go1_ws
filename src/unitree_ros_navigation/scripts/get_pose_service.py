#! /usr/bin/env python3
 
import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

robot_pose = Pose()

def service_callback(request):
  rospy.loginfo("Robot Pose: " + str(robot_pose))

  return EmptyResponse()

def sub_callback(msg):
  global robot_pose
  robot_pose = msg.pose.pose

rospy.init_node('get_pose_service')
my_service = rospy.Service('get_pose_service', Empty, service_callback)
sub_pose = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, sub_callback)

rospy.spin()
