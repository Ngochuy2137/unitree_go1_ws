#!/usr/bin/env python3
import rospy
import math
import tf
import sys
import time
from geometry_msgs.msg import Twist

sys.path.append('/home/server-huynn/workspace/robot_catching_project/trajectory_prediction/go1-control-rocat/unitree_go1_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/lib/python/amd64')
import robot_interface as sdk
from python_utils import printer


HIGHLEVEL = 0xee
RATE = 20  # Loop rate
TRANS_THRES = 0.4  # Meters
ROT_THRES = math.radians(20)  # 20 degrees in radians
ROBOT_TF_FRAME = "dog_frame"
TARGET_TF_FRAME = 'chip_star_frame' # 'chip_star_frame' 'pred_impact_point_frame'
LIN_VEL_SCALING = 1.0
ROT_VEL_SCALING = 1.0

class RobotPIDController:
    def __init__(self, robot_ip="192.168.123.161"):
        rospy.init_node('dynamic_yaw_correction', anonymous=True)
        self.global_printer = printer.Printer()
        self.tf_listener = tf.TransformListener()
        self.mission_complete = False
        self.robot_ip = robot_ip
        self.udp = sdk.UDP(HIGHLEVEL, 8080, self.robot_ip, 8082)
        self.cmd = sdk.HighCmd()
        self.udp.InitCmdData(self.cmd)
        self.velocity_pub = rospy.Publisher("/check/cmd_vel", Twist, queue_size=10)

    def shutdown_node(self):
        rospy.loginfo("Shutting down the node...")
        rospy.signal_shutdown("User requested shutdown")

    def publish_velocity(self, vx, wz):
        vel_msg = Twist()
        vel_msg.linear.x = vx
        vel_msg.angular.z = wz
        self.velocity_pub.publish(vel_msg)

    def calculate_relative_position(self):
        try:
            now = rospy.Time(0)
            self.tf_listener.waitForTransform(ROBOT_TF_FRAME, TARGET_TF_FRAME, now, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(ROBOT_TF_FRAME, TARGET_TF_FRAME, now)
            dx, dy = trans[0], trans[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            desired_yaw = math.atan2(dy, dx)
            rospy.loginfo(f"Distance: {distance:.2f} m, Desired yaw: {math.degrees(desired_yaw):.2f}°")
            return distance, desired_yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF Exception: {str(e)}")
            return None, None

    def send_udp_message(self, forward_velocity, yaw_rate):
        print(f"Sending UDP: {forward_velocity}, {yaw_rate}")
        self.cmd.mode = 2
        self.cmd.gaitType = 1
        self.cmd.velocity = [forward_velocity, 0.0]
        self.cmd.yawSpeed = yaw_rate
        self.cmd.footRaiseHeight = 0.08
        self.cmd.bodyHeight = 0.0
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    def get_robot_state(self):
        state = sdk.LowState()
        self.udp.GetRecv(state)
        if state:
            print(f"Robot State: {state}")
        else:
            print("No response from robot!")

    def lay_down_robot(self):
        rospy.loginfo("Laying down the robot...")
        self.cmd.mode = 5
        self.cmd.gaitType = 0
        self.cmd.velocity = [0.0, 0.0]
        self.cmd.yawSpeed = 0.0
        self.cmd.footRaiseHeight = 0.0
        self.cmd.bodyHeight = -0.2
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    def process_movement(self):
        if self.mission_complete:
            return
        
        distance, desired_yaw = self.calculate_relative_position()
        if distance is None or desired_yaw is None:
            return
        
        if distance < TRANS_THRES:
            rospy.loginfo("Target reached. Stopping robot.")
            self.mission_complete = True
            self.send_udp_message(0.0, 0.0)
            self.lay_down_robot()
            return
        
        if abs(desired_yaw) > ROT_THRES:
            forward_velocity = 0.0
            yaw_rate = max(min(desired_yaw, 0.5), -0.5)
        else:
            forward_velocity = max(min(0.4 - abs(desired_yaw) * 0.8, 0.4), 0.1) * LIN_VEL_SCALING
            yaw_rate = max(min(desired_yaw, 0.5), -0.5) * ROT_VEL_SCALING
        
        self.send_udp_message(forward_velocity, yaw_rate)
        self.publish_velocity(forward_velocity, yaw_rate)
        self.get_robot_state()

    def run(self):
        rate = rospy.Rate(RATE)
        time_start = time.time()
        
        while not rospy.is_shutdown():
            self.global_printer.print_green(f"Control rate: {1 / (time.time() - time_start):.2f}")
            time_start = time.time()
            self.process_movement()
            
            if self.mission_complete:
                print("\n--- Mission Complete ---")
                print(f"Time run: {time.time() - time_start:.2f} s")
                self.shutdown_node()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = RobotPIDController()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted, shutting down.")
