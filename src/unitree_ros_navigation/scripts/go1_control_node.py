#!/usr/bin/env python3
import rospy
import math
import tf
import sys
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

sys.path.append('/home/server-huynn/workspace/robot_catching_project/trajectory_prediction/go1-control-rocat/unitree_go1_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/lib/python/amd64')
# sys.path.append('/home/server-huynn/workspace/robot_catching_project/trajectory_prediction/go1-control-rocat/unitree_go1_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/lib/python/arm64')
import robot_interface as sdk
from python_utils import printer
import copy
import tf.transformations as tf_trans
import numpy as np
from std_msgs.msg import Float32
from python_utils import printer

global_printer = printer.Printer()

HIGHLEVEL = 0xee
RATE = 100  # Loop rate
TRANS_THRES = 0.2  # Meters
ROT_THRES = math.radians(30)  # 20 degrees in radians
# ROBOT_TF_FRAME = "dog_frame"
# TARGET_TF_FRAME = 'chip_star_frame' # 'chip_star_frame' 'pred_impact_point_frame'

ROBOT_POSE_TOPIC = "/mocap_pose_topic/dog_pose"
TARGET_POSE_TOPIC = "NAE/impact_point"  # NAE/impact_point  /mocap_pose_topic/chip_star_pose
LIN_VEL_SCALING = 3.0
ROT_VEL_SCALING = 2.0
GAIT_TYPE = 2
MSG_TIMEOUT = 0.2
MODIFY_Z_UP = True
DEBUG = False
DUMP_RUN_TIME = 0.1
DUMP_RUN_VEL = 1.0

def shutdown_node():
    rospy.loginfo("FORCE Shutting down the node...")
    rospy.signal_shutdown("User requested shutdown")

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
        self.robot_pose:PoseStamped = None
        self.target_pose = None

        self.velocity_pub = rospy.Publisher("/check/cmd_vel", Twist, queue_size=10)
        self.robot_pose_sub = rospy.Subscriber(ROBOT_POSE_TOPIC, PoseStamped, self.robot_pose_callback)
        self.target_pose_sub = rospy.Subscriber(TARGET_POSE_TOPIC, PoseStamped, self.target_pose_callback)
        self.new_robot_pose_pub = rospy.Publisher("/check/robot_pose", PoseStamped, queue_size=10)
        self.new_target_pose_pub = rospy.Publisher("/check/target_pose", PoseStamped, queue_size=10)

        self.last_robot_pose_time = time.time()
        self.last_target_pose_time = time.time()

        self.event_robot_pose = None
        self.event_time = None
        self.got_first_target_event = False


    def robot_pose_callback(self, msg):
        """ Xử lý dữ liệu Pose cho robot """
        self.robot_pose = copy.deepcopy(msg)
        
        if MODIFY_Z_UP:
            # Chuyển đổi vị trí
            self.robot_pose.pose.position.y = -msg.pose.position.z
            self.robot_pose.pose.position.z = msg.pose.position.y

            # Lấy quaternion gốc
            q_orig = msg.pose.orientation
            q_new = [q_orig.x, -q_orig.z, q_orig.y, q_orig.w]  # Hoán đổi các trục phù hợp

            # Chuẩn hóa quaternion để tránh sai số
            q_new = tf_trans.unit_vector(q_new)

            # Cập nhật quaternion mới
            self.robot_pose.pose.orientation.x = q_new[0]
            self.robot_pose.pose.orientation.y = q_new[1]
            self.robot_pose.pose.orientation.z = q_new[2]
            self.robot_pose.pose.orientation.w = q_new[3]

    def target_pose_callback(self, msg: PoseStamped):
        """ Xử lý dữ liệu Pose cho mục tiêu """
        self.target_pose = copy.deepcopy(msg)
        # time_msg = msg.header.stamp.to_sec()
        # time_ros_now = rospy.Time.now().to_sec()
        # time_diff = time_ros_now - time_msg
        # print(f'time_diff: {time_diff}')
        if MODIFY_Z_UP:
            # Chuyển đổi vị trí
            self.target_pose.pose.position.y = -msg.pose.position.z
            self.target_pose.pose.position.z = msg.pose.position.y

            # Lấy quaternion gốc
            q_orig = msg.pose.orientation
            q_new = [q_orig.x, -q_orig.z, q_orig.y, q_orig.w]  # Hoán đổi các trục phù hợp

            # Chuẩn hóa quaternion
            q_new = tf_trans.unit_vector(q_new)

            # Cập nhật quaternion mới
            self.target_pose.pose.orientation.x = q_new[0]
            self.target_pose.pose.orientation.y = q_new[1]
            self.target_pose.pose.orientation.z = q_new[2]
            self.target_pose.pose.orientation.w = q_new[3]

    def is_pose_timeout(self):
        if time.time() - self.last_robot_pose_time > MSG_TIMEOUT:
            self.robot_pose = None
            return True
        if time.time() - self.last_target_pose_time > MSG_TIMEOUT:
            self.target_pose = None
            return True
        return False
        
    def shutdown_node(self):
        rospy.loginfo("Shutting down the node...")
        rospy.signal_shutdown("User requested shutdown")

    def publish_velocity(self, vx, wz):
        vel_msg = Twist()
        vel_msg.linear.x = vx
        vel_msg.angular.z = wz
        self.velocity_pub.publish(vel_msg)

    def get_yaw_from_pose(self, pose):
        from tf.transformations import euler_from_quaternion
        """Chuyển quaternion thành góc yaw (quay quanh trục Z)"""
        orientation_q = pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(quaternion)  # Lấy yaw
        return yaw

    def calculate_relative_position(self, robot_pose: PoseStamped, target_pose: PoseStamped):
        if robot_pose is None or target_pose is None:
            rospy.logwarn("No pose message received.")
            return None, None

        # Lấy tọa độ x, y từ PoseStamped
        x_r, y_r = robot_pose.pose.position.x, robot_pose.pose.position.y
        x_t, y_t = target_pose.pose.position.x, target_pose.pose.position.y

        if not self.got_first_target_event:
            self.event_robot_pose = np.array([robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z])
            self.event_time = time.time()
            # log warn
            rospy.logwarn("Event robot pose received.")
            self.got_first_target_event = True

        # check if robot pose is different from the event pose
        if self.event_robot_pose is not None:
            robot_pose_np = np.array([robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z])
            equal = np.allclose(self.event_robot_pose, robot_pose_np, atol=0.001)
            if not equal:
                time_delay = time.time() - self.event_time
                global_printer.print_blue(f"Reaction delay: {time_delay} s")
                if target_pose.header.frame_id is str:
                    fly_time_start = float(target_pose.header.frame_id)
                    global_printer.print_blue(f'Preparation time: {(rospy.Time.now().to_nsec() - fly_time_start) / 1e9}')
                global_printer.print_green(f"Robot moving. Reaction delay: {time_delay} s")
                # shutdown_node()
                self.event_robot_pose = None
            else:
                global_printer.print_yellow("Robot standing still")


        if DEBUG: print(f'robot_pose x, y: {x_r}, {y_r}')
        if DEBUG: print(f'target_pose x, y: {x_t}, {y_t}')

        # Lấy góc yaw của robot trong world
        yaw_robot = self.get_yaw_from_pose(robot_pose)

        # Tính dx, dy trong world
        dx_world = x_t - x_r
        dy_world = y_t - y_r

        if DEBUG: print(f'dx (world): {dx_world}, dy (world): {dy_world}')

        # Chuyển về hệ tọa độ của robot bằng cách xoay ngược lại
        R_inv = np.array([[math.cos(yaw_robot), math.sin(yaw_robot)],
                        [-math.sin(yaw_robot), math.cos(yaw_robot)]])  # R^-1 = R^T với ma trận quay

        dx_robot, dy_robot = np.dot(R_inv, np.array([dx_world, dy_world]))

        if DEBUG: print(f'dx (robot): {dx_robot}, dy (robot): {dy_robot}')

        # Tính khoảng cách và góc yaw mong muốn trong hệ robot
        distance = math.sqrt(dx_robot ** 2 + dy_robot ** 2)
        desired_yaw = math.atan2(dy_robot, dx_robot)

        if DEBUG: rospy.loginfo(f"Distance: {distance:.2f} m, Desired yaw: {math.degrees(desired_yaw):.2f}°")
        return distance, desired_yaw


    def send_udp_message(self, forward_velocity, angular_velocity):
        # print(f"Sending UDP: {forward_velocity}, {angular_velocity}")
        self.cmd.mode = 2
        self.cmd.gaitType = GAIT_TYPE
        self.cmd.velocity = [forward_velocity, 0.0]
        self.cmd.yawSpeed = angular_velocity
        self.cmd.footRaiseHeight = 0.08
        self.cmd.bodyHeight = 0.0
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    def receive_udp_robot_state(self):
        state = sdk.LowState()
        self.udp.GetRecv(state)
        d = {'FR_0':0, 'FR_1':1, 'FR_2':2,
         'FL_0':3, 'FL_1':4, 'FL_2':5, 
         'RR_0':6, 'RR_1':7, 'RR_2':8, 
         'RL_0':9, 'RL_1':10, 'RL_2':11 }
        if state:
            a = state.motorState[d['FR_0']].q
            print(f"Robot State: {a}")
        else:
            print("No response from robot!")

    # def get_robot_state(self):
    #     state = sdk.HighState()
    #     self.udp.GetRecv(state)
    #     if state:
    #         # In ra các thông tin trạng thái cần thiết
    #         print('state:', state)
    #         print(f"Roll: {state.imu.rpy[0]:.2f}, Pitch: {state.imu.rpy[1]:.2f}, Yaw: {state.imu.rpy[2]:.2f}")
    #         print(f"Forward Speed: {state.velocity} m/s")
    #         # print(f"Battery Voltage: {state.battery:.2f} V")
    #     else:
    #         print("No response from robot!")

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

    def dump_run(self, time_start, time_run, vel):
        if time.time() - time_start < DUMP_RUN_TIME:
            self.send_udp_message(DUMP_RUN_VEL, 0.0)
            self.publish_velocity(DUMP_RUN_VEL, 0.0)
            print('Dump run')
        else:
            pass

    def process_movement(self, exp_time_start):
        if self.mission_complete:
            return None, None
        if self.target_pose is None:
            self.dump_run(exp_time_start, DUMP_RUN_TIME, DUMP_RUN_VEL)
            return None, None
        
        distance, desired_yaw = self.calculate_relative_position(self.robot_pose, self.target_pose)
        if distance is None or desired_yaw is None:
            return None, None
        
        if distance < TRANS_THRES:
            rospy.loginfo("Target reached. Stopping robot.")
            self.mission_complete = True
            self.send_udp_message(0.0, 0.0)
            # self.lay_down_robot()
            return None, None
        
        if abs(desired_yaw) > ROT_THRES:
            forward_velocity = 0.0
            angular_velocity = max(min(desired_yaw, 0.5), -0.5)
        else:
            forward_velocity = max(min(0.4 - abs(desired_yaw) * 0.8, 0.4), 0.1) * LIN_VEL_SCALING   # lim in range [0.1, 0.4]
            angular_velocity = max(min(desired_yaw, 0.5), -0.5) * ROT_VEL_SCALING   # lim in range [-0.5, 0.5]

            # forward_velocity = max(min(0.4 - abs(desired_yaw) * 0.8, 0.4), 0.1)   # lim in range [0.1, 0.4]
            # angular_velocity = max(min(desired_yaw, 0.5), -0.5)   # lim in range [-0.5, 0.5]
        
        print('forward_velocity: ', forward_velocity)
        self.send_udp_message(forward_velocity, angular_velocity)
        self.publish_velocity(forward_velocity, angular_velocity)
        # return just for debugging
        return forward_velocity, angular_velocity

    def cal_avg_vel(self, velocities, timestamps):
        # Kiểm tra tính hợp lệ của dữ liệu đầu vào
        if len(timestamps) == len(velocities):
            velocities = velocities[1:]
        else:
            raise ValueError("Số lượng vận tốc và thời gian không khớp nhau.")
        # Tính các khoảng thời gian delta_t
        delta_t = [timestamps[i+1] - timestamps[i] for i in range(len(velocities))]
        # Tính tổng tích vận tốc-thời gian và tổng các khoảng thời gian
        weighted_sum = sum(v * dt for v, dt in zip(velocities, delta_t))
        total_time = sum(delta_t)
        # Kiểm tra tránh chia cho 0
        if total_time == 0:
            raise ValueError("Tổng thời gian bằng 0, không thể tính vận tốc trung bình.")
        # Tính vận tốc trung bình
        average_velocity = weighted_sum / total_time
        return average_velocity
    
    def run(self):
        rate = rospy.Rate(RATE)
        exp_time_start = time.time()
        
        vel_list = []
        time_list = []

        while self.robot_pose is None:
            # sleep to wait for robot pose
            rate.sleep()

        robot_pos_start = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z])
        while not rospy.is_shutdown():
            if DEBUG: self.global_printer.print_green(f"Control rate: {1 / (time.time() - exp_time_start):.2f}")
            if DEBUG: self.receive_udp_robot_state()
            vx, wz = self.process_movement(exp_time_start)
            
            # just for debugging
            if vx is not None and wz is not None:
                vel_list.append(vx)
                time_list.append(time.time())
            if self.mission_complete:
                print("\n-------- Mission Complete --------")
                robot_pos_stop = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z])
                time_run = time.time() - exp_time_start
                print(f"Time run: {time_run:.6f} s")
                dis_run = np.linalg.norm(robot_pos_start - robot_pos_stop)  # calculate distance
                print(f'Dis run: {dis_run}')
                print(f'Real avg vel: {dis_run / (time_run):.6f} m/s')
                print(f"Command avg velocity: {self.cal_avg_vel(vel_list, time_list):.2f} m/s")
                print('----------------------------------\n')
                self.shutdown_node()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = RobotPIDController()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted, shutting down.")
