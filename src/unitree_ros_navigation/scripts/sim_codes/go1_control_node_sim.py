#!/usr/bin/env python3
import rospy
import math
import tf
import sys
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

# sys.path.append('/home/huynn/huynn_ws/robot_catching_ws/unitree_go1_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/lib/python/arm64')
# import robot_interface as sdk
from python_utils import printer
import copy
import tf.transformations as tf_trans
import numpy as np
from std_msgs.msg import Float32

global_printer = printer.Printer()

HIGHLEVEL = 0xee
RATE = 50  # Loop rate
TRANS_THRES = 0.2  # Meters
ROT_THRES = math.radians(30)  # 20 degrees in radians
# ROBOT_TF_FRAME = "dog_frame"
# TARGET_TF_FRAME = 'chip_star_frame' # 'chip_star_frame' 'pred_impact_point_frame'

ROBOT_POSE_TOPIC = "unitree_go1/pose"
TARGET_POSE_TOPIC = "NAE/impact_point"  # NAE/impact_point  /mocap_pose_topic/chip_star_pose
TRIGGER_DUMP_RUN_TOPIC = "/mocap_pose_topic/chip_star_pose"
LIN_VEL_SCALING = 2.0
ROT_VEL_SCALING = 2.0
GAIT_TYPE = 2
DIS_XY_THRES = 0.05

# PID_X = [2.5, 0.0, 0.1]
# PID_Y = [1.5, 0.0, 0.1]
# PID_THETA = [2.0, 0.0, 0.1]

PID_X = [7, 0.01, 0.05]
PID_Y = [4, 0.01, 0.05]
PID_THETA = [2.0, 0.0, 0.1]

MSG_TIMEOUT = 0.2
# MODIFY_Z_UP = False
MODIFY_Z_UP_PREDICT = True
MODIFY_Z_UP_ROBOT_POSE = False

DEBUG = False
DUMP_RUN_TIME = 0.5
DUMP_RUN_VEL = 2.0
VXRANGE = [-0.5, 0.5]
VYRANGE = [-0.5, 0.5]
WZRANGE = [-1.0, 1.0]
ACTIVE_ZONE_X = [-10, 10]
ACTIVE_ZONE_Y = [-10, 10]

NO_CONTROL = False

def shutdown_node():
    rospy.loginfo("FORCE Shutting down the node...")
    rospy.signal_shutdown("User requested shutdown")


class PIDController:
    def __init__(self, 
                Kp_x=1.0, Ki_x=0.0, Kd_x=0.1, 
                Kp_y=1.5, Ki_y=0.0, Kd_y=0.1,
                Kp_theta=2.0, Ki_theta=0.0, Kd_theta=0.1,
                vx_range=(-2.3, 3.3), vy_range=(-1.0, 1.0), wz_range=(-2, 2),
                integral_limit=1.0, deadband_xytheta=(0.05, 0.05, 3*math.pi/180),
                boost_x_enabled=False):

        # Tham số PID cho X, Y, và Theta (góc quay)
        self.Kp_x, self.Ki_x, self.Kd_x = Kp_x, Ki_x, Kd_x
        self.Kp_y, self.Ki_y, self.Kd_y = Kp_y, Ki_y, Kd_y
        self.Kp_theta, self.Ki_theta, self.Kd_theta = Kp_theta, Ki_theta, Kd_theta

        # Giới hạn vận tốc
        self.vx_range = vx_range
        self.vy_range = vy_range
        self.wz_range = wz_range

        self.integral_limit = integral_limit
        self.deadband_x, self.deadband_y, self.deadband_theta = deadband_xytheta
        self.boost_x_enabled = boost_x_enabled  # Bật/tắt tính năng boost trục X

        # Trạng thái PID
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_theta = 0.0

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_theta = 0.0

    def calculate(self, current_pos, goal_pos, current_quat, dt):
        # Vị trí hiện tại và vị trí đích
        x_r, y_r = current_pos
        x_g, y_g = goal_pos

        # Chuyển quaternion của robot sang yaw (hướng robot)
        theta_r = self.quaternion_to_yaw(current_quat)

        # ===== Chuyển đổi từ absolute → relative coordinates =====
        delta_x = x_g - x_r
        delta_y = y_g - y_r

        # Sai số vị trí trong hệ tọa độ của robot
        error_x = math.cos(theta_r) * delta_x + math.sin(theta_r) * delta_y
        error_y = -math.sin(theta_r) * delta_x + math.cos(theta_r) * delta_y

        # Sai số góc (robot hướng về mục tiêu)
        error_theta = self.normalize_angle(math.atan2(delta_y, delta_x) - theta_r)

        # ===== Deadband để bỏ qua sai số nhỏ =====
        if abs(error_y) < self.deadband_y:
            error_y = 0.0
        if abs(error_x) < self.deadband_x:
            error_x = 0.0

        dis_xy = math.sqrt(error_x**2 + error_y**2)
        if abs(error_theta) < self.deadband_theta or dis_xy<0.3:  # Deadband cho sai số góc (2 độ)
            error_theta = 0.0

        # ===== PID cho trục Y =====
        self.integral_y += error_y * dt
        self.integral_y = max(min(self.integral_y, self.integral_limit), -self.integral_limit)
        derivative_y = (error_y - self.prev_error_y) / dt if dt > 0 else 0.0
        vy = (self.Kp_y * error_y) + (self.Ki_y * self.integral_y) + (self.Kd_y * derivative_y)

        # ===== PID cho trục X (có thể bật/tắt boost) =====
        self.integral_x += error_x * dt
        self.integral_x = max(min(self.integral_x, self.integral_limit), -self.integral_limit)
        derivative_x = (error_x - self.prev_error_x) / dt if dt > 0 else 0.0

        # Bật/tắt tăng cường điều khiển X khi Y ổn định
        if self.boost_x_enabled:
            Kp_x_boost = self.Kp_x * 1.5 if abs(error_y) < 0.05 else self.Kp_x
        else:
            Kp_x_boost = self.Kp_x

        vx = (Kp_x_boost * error_x) + (self.Ki_x * self.integral_x) + (self.Kd_x * derivative_x)

        # ===== PID cho góc quay (Theta) =====
        self.integral_theta += error_theta * dt
        self.integral_theta = max(min(self.integral_theta, self.integral_limit), -self.integral_limit)
        derivative_theta = (error_theta - self.prev_error_theta) / dt if dt > 0 else 0.0
        wz = (self.Kp_theta * error_theta) + (self.Ki_theta * self.integral_theta) + (self.Kd_theta * derivative_theta)

        # ===== Giới hạn vận tốc =====
        # vx = vx**LIN_VEL_SCALING
        # vy = vy**LIN_VEL_SCALING
        # wz = wz**ROT_VEL_SCALING
        vx = max(min(vx, self.vx_range[1]), self.vx_range[0])
        vy = max(min(vy, self.vy_range[1]), self.vy_range[0])
        wz = max(min(wz, self.wz_range[1]), self.wz_range[0])

        # Cập nhật sai số trước đó
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_theta = error_theta

        return vx, vy, wz

    def reset(self):
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_theta = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_theta = 0.0

    @staticmethod
    def normalize_angle(angle):
        """
        Chuẩn hóa góc về khoảng [-π, π].
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def quaternion_to_yaw(q):
        """
        Chuyển quaternion sang yaw (radian).

        Args:
            q (tuple): Quaternion (x, y, z, w)

        Returns:
            float: Góc yaw (radian)
        """
        x, y, z, w = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

class RobotController:
    def __init__(self, robot_ip="192.168.123.161"):
        rospy.init_node('robot_pid_high_level_controller', anonymous=True)
        self.global_printer = printer.Printer()
        self.tf_listener = tf.TransformListener()
        self.mission_complete = False
        self.robot_ip = robot_ip
        # self.udp = sdk.UDP(HIGHLEVEL, 8080, self.robot_ip, 8082)
        # self.cmd = sdk.HighCmd()
        # self.udp.InitCmdData(self.cmd)
        self.robot_pose:PoseStamped = None
        self.target_pose:PoseStamped = None
        self.trigger_dump_run = False
        self.already_trigger_dump_run = False

        self.velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.robot_pose_sub = rospy.Subscriber(ROBOT_POSE_TOPIC, Odometry, self.robot_pose_callback)
        self.target_pose_sub = rospy.Subscriber(TARGET_POSE_TOPIC, PoseStamped, self.target_pose_callback)
        self.trigger_dump_run_sub = rospy.Subscriber(TRIGGER_DUMP_RUN_TOPIC, PoseStamped, self.trigger_pose_callback)
        self.new_robot_pose_pub = rospy.Publisher("/check/robot_pose", PoseStamped, queue_size=10)
        self.new_target_pose_pub = rospy.Publisher("/check/target_pose", PoseStamped, queue_size=10)

        self.last_robot_pose_time = time.time()
        self.last_target_pose_time = time.time()

        self.event_robot_pose = None
        self.event_time = None
        self.got_first_target_event = False

        # self.pid = PIDController(Kp=KP, Ki=KI, Kd=KD, vx_range=(-2.3, 3.3), vy_range=(-1.0, 1.0))
        # self.pid = PIDController(Kp_x=1.0, Ki_x=0.0, Kd_x=0.1, 
        #                         Kp_y=1.5, Ki_y=0.0, Kd_y=0.1,
        #                         Kp_theta=2.0, Ki_theta=0.0, Kd_theta=0.1,
        #                         vx_range=(-2.3, 3.3), vy_range=(-1.0, 1.0), wz_range=(-2, 2),
        #                         integral_limit=1.0, deadband=0.01)
        self.pid = PIDController(Kp_x=PID_X[0], Ki_x=PID_X[1], Kd_x=PID_X[2],
                                Kp_y=PID_Y[0], Ki_y=PID_Y[1], Kd_y=PID_Y[2],
                                Kp_theta=PID_THETA[0], Ki_theta=PID_THETA[1], Kd_theta=PID_THETA[2],
                                vx_range=VXRANGE, vy_range=VYRANGE, wz_range=WZRANGE,
                                integral_limit=1.0, deadband_xytheta=(0.05, 0.05, 3*math.pi/180))

        # active zone
        self.active_zone_x = ACTIVE_ZONE_X
        self.active_zone_y = ACTIVE_ZONE_Y

        self.dump_run_trigger_zone_x = [-2.5, 3.5]
        self.dump_run_trigger_zone_y = [-2.0, 0.5]

        self.tc1 = None # 1st seen object (trigger) time
        self.pos_tc1 = None # robot pos corresponding to tc1
        self.tc2 = None # robot 1st move
        self.tc3 = []       # impact point sub time 
        self.pos_tc3 = []   # impact point pos corresponding to tc3
        self.tc34 = []       # predicted impact time


    def robot_pose_callback(self, msg: Odometry):
        """ Xử lý dữ liệu Pose cho robot """
        cur_pose = copy.deepcopy(msg)
        self.robot_pose = PoseStamped()
        self.robot_pose.header.frame_id = 'world'
        self.robot_pose.header.stamp = rospy.Time.now()
        self.robot_pose.pose = cur_pose.pose.pose
        
        if MODIFY_Z_UP_ROBOT_POSE:
            # Chuyển đổi vị trí
            self.robot_pose.pose.position.y = -cur_pose.pose.pose.position.z
            self.robot_pose.pose.position.z = cur_pose.pose.pose.position.y

            # Lấy quaternion gốc
            q_orig = cur_pose.pose.pose.orientation
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
        if MODIFY_Z_UP_PREDICT:
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
        global_printer.print_yellow(f'New target pose received: {self.target_pose.pose.position.x}, {self.target_pose.pose.position.y}')

    def trigger_pose_callback(self, msg: PoseStamped):
        """ Xử lý dữ liệu Pose cho mục tiêu """
        if not self.already_trigger_dump_run:
        
            object_pose = copy.deepcopy(msg)
            if MODIFY_Z_UP_PREDICT:
                # Chuyển đổi vị trí
                object_pose_x = object_pose.pose.position.x
                object_pose_y = -msg.pose.position.z
            else:
                object_pose_x = object_pose.pose.position.x
                object_pose_y = object_pose.pose.position.y
            object_pose_z = msg.pose.position.y
            if object_pose_x >= self.dump_run_trigger_zone_x[0] and object_pose_x <= self.dump_run_trigger_zone_x[1] and \
                object_pose_y >= self.dump_run_trigger_zone_y[0] and object_pose_y <= self.dump_run_trigger_zone_y[1]:

                self.trigger_dump_run = True
                self.dump_run_time_start = time.time()
                global_printer.print_green('Trigger dump run, becareful !')
                self.already_trigger_dump_run = True
                self.dump_run(self.dump_run_time_start, DUMP_RUN_TIME, DUMP_RUN_VEL)
                print('TRIGGER POS: ', object_pose_x, object_pose_y, object_pose_z)
                if object_pose_x - self.dump_run_trigger_zone_x[0] > 0.2:
                    global_printer.print_red('Trigger moment is too late, becareful !')
                # shutdown_node()
            else:
                self.trigger_dump_run = False

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

    def publish_velocity(self, vx, vy, wz):
        vel_msg = Twist()
        vel_msg.linear.x = vx
        vel_msg.linear.y = vy
        vel_msg.angular.z = wz
        self.velocity_pub.publish(vel_msg)

    def get_yaw_from_pose(self, pose):
        from tf.transformations import euler_from_quaternion
        """Chuyển quaternion thành góc yaw (quay quanh trục Z)"""
        orientation_q = pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(quaternion)  # Lấy yaw
        return yaw

    # def send_udp_message(self, vx, vy, wz):
    #     if NO_CONTROL:
    #         return
    #     # print(f"Sending UDP: {forward_velocity}, {angular_velocity}")
    #     self.cmd.mode = 2
    #     self.cmd.gaitType = GAIT_TYPE
    #     self.cmd.velocity = [vx, vy]
    #     self.cmd.yawSpeed = wz
    #     self.cmd.footRaiseHeight = 0.08
    #     self.cmd.bodyHeight = 0.0
    #     self.udp.SetSend(self.cmd)
    #     self.udp.Send()

    # def receive_udp_robot_state(self):
    #     state = sdk.LowState()
    #     self.udp.GetRecv(state)
    #     d = {'FR_0':0, 'FR_1':1, 'FR_2':2,
    #      'FL_0':3, 'FL_1':4, 'FL_2':5, 
    #      'RR_0':6, 'RR_1':7, 'RR_2':8, 
    #      'RL_0':9, 'RL_1':10, 'RL_2':11 }
    #     if state:
    #         a = state.motorState[d['FR_0']].q
    #         print(f"Robot State: {a}")
    #     else:
    #         print("No response from robot!")

    # def lay_down_robot(self):
    #     rospy.loginfo("Laying down the robot...")
    #     self.cmd.mode = 5
    #     self.cmd.gaitType = 0
    #     self.cmd.velocity = [0.0, 0.0]
    #     self.cmd.yawSpeed = 0.0
    #     self.cmd.footRaiseHeight = 0.0
    #     self.cmd.bodyHeight = -0.2
    #     self.udp.SetSend(self.cmd)
    #     self.udp.Send()

    def dump_run(self, time_start, time_run, vel_max):
        delta_t = time.time() - time_start
        if delta_t < time_run:
            vx = vel_max/(delta_t*0.5)
            vx = max(min(vx, vel_max), 0.1)
            # self.send_udp_message(vx, 0.0, 0.0)
            self.publish_velocity(vx, 0.0, 0.0)
            print('Dump run - time: ', delta_t)
        else:
            pass

    def process_movement(self, exp_time_start, last_time):
        # check if in active zone
        # if self.robot_pose is None or \
        if  self.robot_pose.pose.position.x < self.active_zone_x[0] or \
            self.robot_pose.pose.position.x > self.active_zone_x[1] or \
            self.robot_pose.pose.position.y < self.active_zone_y[0] or \
            self.robot_pose.pose.position.y > self.active_zone_y[1]:
            global_printer.print_red(f"Robot out of active zone. Robot xy: {self.robot_pose.pose.position.x}, {self.robot_pose.pose.position.y}")
            start_pub_time = time.time()
            while time.time() - start_pub_time < 3.0:
                # self.send_udp_message(0.0, 0.0, 0.0)
                self.publish_velocity(0.0, 0.0, 0.0)
            self.mission_complete = True
            return None, None

        if self.target_pose is None and self.trigger_dump_run==True:
            self.dump_run(self.dump_run_time_start, DUMP_RUN_TIME, DUMP_RUN_VEL)
            # self.target_pose = PoseStamped()
            # self.target_pose.pose.position.x = 2.0
            # self.target_pose.pose.position.y = -1.0
            # self.target_pose.pose.position.z = 0.1
            # self.target_pose.pose.orientation.z = 1.0
            return None, None
        if self.robot_pose is None or self.target_pose is None:
            # rospy.logwarn("No pose message received.")
            return None, None

        # print('\n-----')

        delta_t = time.time() - last_time
        robot_pos = [self.robot_pose.pose.position.x, self.robot_pose.pose.position.y]
        robot_quat = [self.robot_pose.pose.orientation.x, self.robot_pose.pose.orientation.y, self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w]
        goal_pos = [self.target_pose.pose.position.x, self.target_pose.pose.position.y]
        dis_xy = math.sqrt((robot_pos[0] - goal_pos[0])**2 + (robot_pos[1] - goal_pos[1])**2)
        if dis_xy <= DIS_XY_THRES:
            self.mission_complete = True
            # self.send_udp_message(0.0, 0.0, 0.0)
            self.publish_velocity(0.0, 0.0, 0.0)
            return None, None

        # print(f'real hz = {1/(time.time() - last_time):.3f}')
        vx, vy, wz = self.pid.calculate(robot_pos, goal_pos, robot_quat, delta_t)
        # print('forward_velocity: ', forward_velocity)
        # print(f'    vx: {vx}, vy: {vy}, wz: {wz*180/np.pi:.3f}')
        wz = 0
        # self.send_udp_message(vx, vy, wz)
        self.publish_velocity(vx, vy, wz)
        # print('check vx, vy: ', vx, vy)

        # return just for debugging
        return vx, vy

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


        last_time = time.time()
        got_robot_start_pose = False

        global_printer.print_blue('===================================================', background=True)
        global_printer.print_blue('ARE YOU READY ? Press Enter to start the mission...', background=True)
        global_printer.print_blue('===================================================', background=True); input(); input()

        print('Mission start !')

        while not rospy.is_shutdown():
            if DEBUG: self.global_printer.print_green(f"Control rate: {1 / (time.time() - exp_time_start):.2f}")
            # if DEBUG: self.receive_udp_robot_state()
            vx, vy = self.process_movement(exp_time_start, last_time=last_time)
            last_time = time.time()
            
            # just for debugging
            if vx is not None and vy is not None:
                if self.target_pose is not None:
                    dis_xy = math.sqrt((self.robot_pose.pose.position.x - self.target_pose.pose.position.x)**2 + (self.robot_pose.pose.position.y - self.target_pose.pose.position.y)**2)
                    if dis_xy <=1.0:
                        vel_list.append(vx)
                        time_list.append(time.time())
                        if not got_robot_start_pose:
                            robot_pos_start_cons = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z])
                            time_start_cons = time.time()
                            got_robot_start_pose = True

            if self.mission_complete:
                print("\n-------- Mission Complete --------")
                robot_pos_stop = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z])
                time_run = time.time() - time_start_cons
                print(f"Time run: {time_run:.6f} s")
                dis_run = np.linalg.norm(robot_pos_start_cons - robot_pos_stop)  # calculate distance
                print(f'Dis run: {dis_run}')
                print(f'Real avg vel: {dis_run / (time_run):.6f} m/s')
                print(f"Command avg velocity: {self.cal_avg_vel(vel_list, time_list):.2f} m/s")
                print('----------------------------------\n')
                self.shutdown_node()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = RobotController()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted, shutting down.")
