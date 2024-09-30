#! /usr/bin/env python3
 
import rospy
from std_srvs.srv import Empty, EmptyRequest
import sys 
 
rospy.init_node('service_client')
rospy.wait_for_service('/global_localization')

disperse_particles_service = rospy.ServiceProxy('/global_localization', Empty)
msg = EmptyRequest()
result = disperse_particles_service(msg)

rospy.loginfo(result)

