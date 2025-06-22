#!/usr/bin/env python3
import rospy
import math
import tf
import sys
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from python_utils.singer import warn_beep, beep

from python_utils import printer
import copy
import tf.transformations as tf_trans
import numpy as np
from std_msgs.msg import Float32

global_printer = printer.Printer()

HIGHLEVEL = 0xee
DEBUG = False
NO_CONTROL = False



# load params from server
RATE = rospy.get_param("high_level_controller/rate")
GAIT_TYPE = rospy.get_param("high_level_controller/gait_type")
ROBOT_POSE_Z_UP_TOPIC = rospy.get_param("robot_pose_z_up_topic")
TARGET_POSE_Z_UP_TOPIC = rospy.get_param("predicted_impact_point_z_up_topic")
TRIGGER_DUMMY_RUN_TOPIC = rospy.get_param("trigger_dummy_run_topic")
CTRL_TOLERANCE_XY = rospy.get_param("high_level_controller/control_tolerance_xy")

PID_X = rospy.get_param("high_level_controller/pid_x")
PID_Y = rospy.get_param("high_level_controller/pid_y")
PID_THETA = rospy.get_param("high_level_controller/pid_theta")
DEADBAND_XYTH = rospy.get_param("high_level_controller/deadband_xytheta")

VXRANGE = rospy.get_param("high_level_controller/vx_range")
VYRANGE = rospy.get_param("high_level_controller/vy_range")
WZRANGE = rospy.get_param("high_level_controller/wz_range")

MODIFY_Z_UP_GOAL = rospy.get_param("high_level_controller/modify_z_up_goal")
MODIFY_Z_UP_ROBOT_POSE = rospy.get_param("high_level_controller/modify_z_up_robot_pose")

DUMMY_RUN_TIME = rospy.get_param("high_level_controller/dummy_run_time")
DUMMY_RUN_VEL = rospy.get_param("high_level_controller/dummy_run_vel")

ACTIVE_ZONE_X = rospy.get_param("high_level_controller/active_zone_x")
ACTIVE_ZONE_Y = rospy.get_param("high_level_controller/active_zone_y")

DUMMY_ZONE_X = rospy.get_param("high_level_controller/dummy_zone_x")
DUMMY_ZONE_Y = rospy.get_param("high_level_controller/dummy_zone_y")

MAX_CONTROL_TIME = rospy.get_param("high_level_controller/max_control_time")

def shutdown_node():
    rospy.loginfo("FORCE Shutting down the node...")
    rospy.signal_shutdown("User requested shutdown")

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
def send_robot_reached_goal_srv(data):
    global_printer.print_green("Sending robot reach goal signal to /robot_reached_goal_srv")
    try:
        rospy.wait_for_service('/robot_reached_goal_srv', timeout=2)
    except rospy.ROSException:
        rospy.logerr("Service '/robot_reached_goal_srv' is not available within timeout!")
        return
    try:
        service_client = rospy.ServiceProxy('/robot_reached_goal_srv', SetBool)
        request = SetBoolRequest(data=data)  # Gửi True/False
        response = service_client(data=data)

        print(f"        Response: success={response.success}, message='{response.message}'")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


# scaling_function ví dụ:
def scaling_function(error):
    if error > 0.2:
        return 1.0
    elif error > 0.1:
        return error / 0.5
    else:
        return 0.1  # hoặc nhỏ hơn nữa
    
class PIDController:
    def __init__(self, 
                Kp_x, Ki_x, Kd_x, 
                Kp_y, Ki_y, Kd_y,
                Kp_theta, Ki_theta, Kd_theta,
                vx_range, vy_range, wz_range,
                integral_limit, deadband_xytheta,
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
        derivative_y = (error_y - self.prev_error_y) / dt if dt > 0.001 else 0.0
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
        self.using_real_robot = rospy.get_param("using_real_robot")
        self.no_cmd = rospy.get_param("~no_cmd", False)
        if self.using_real_robot:
            sys.path.append('/home/server-huynn/workspace/robot_catching_project/experiment/unitree_go1_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/lib/python/amd64')
            # sys.path.append('/home/huynn/huynn_ws/robot_catching_ws/unitree_go1_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/lib/python/amd64')
            import robot_interface as sdk
            self.robot_ip = rospy.get_param("robot_ip")
            self.udp = sdk.UDP(HIGHLEVEL, 8080, self.robot_ip, 8082)
            self.cmd = sdk.HighCmd()
            self.udp.InitCmdData(self.cmd)

        # self.tf_listener = tf.TransformListener()
        self.mission_complete = False
        self.robot_ip = robot_ip
        # self.udp = sdk.UDP(HIGHLEVEL, 8080, self.robot_ip, 8082)
        # self.cmd = sdk.HighCmd()
        # self.udp.InitCmdData(self.cmd)

        rospy.Subscriber(ROBOT_POSE_Z_UP_TOPIC, Odometry, self.robot_pose_callback)

        rospy.Subscriber(TRIGGER_DUMMY_RUN_TOPIC, PoseStamped, self.trigger_pose_callback, queue_size=10)
        rospy.Subscriber(TARGET_POSE_Z_UP_TOPIC, PoseStamped, self.target_pose_callback)

        self.velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.new_robot_pose_pub = rospy.Publisher("/check/robot_pose", PoseStamped, queue_size=10)
        self.new_target_pose_pub = rospy.Publisher("/check/target_pose", PoseStamped, queue_size=10)


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
                                integral_limit=1.0, deadband_xytheta=DEADBAND_XYTH)

        # active zone
        self.active_zone_x = ACTIVE_ZONE_X
        self.active_zone_y = ACTIVE_ZONE_Y
        # only trigger dummy run when robot in this zone
        self.dummy_run_trigger_zone_x = DUMMY_ZONE_X
        self.dummy_run_trigger_zone_y = DUMMY_ZONE_Y

        self.robot_pose = None
        self.reset_controller()

        rospy.Service('/ask_if_robot_is_ready_srv', SetBool, self.handle_if_is_free_ask)
        rospy.Service('/stop_control_session_srv', SetBool, self.handle_stop_control_session_srv)

    def reset_controller(self):
        self.publish_velocity(0.0, 0.0, 0.0)
        self.pid.reset()
        
        self.trigger_dummy_run = False
        self.trigger_time = None
        self.robot_init_pos = None

        self.first_move_time = None

        self.target_pose:PoseStamped = None
        self.got_first_target_event = False
                
        self.first_goal_get_time = None
        self.first_goal_get_robot_pose = None

        self.mission_complete = False
        self.robot_is_free = True
        # self.stop_robot_order = False

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
            
    def handle_if_is_free_ask(self, req):
        global_printer.print_green(f"Received service QUESTION /ask_if_robot_is_ready_srv -> {self.robot_is_free}")
        if self.robot_is_free:
            return SetBoolResponse(success=True, message="Robot is free (got no command)")
        else:
            return SetBoolResponse(success=False, message="Robot is busy (got a command)")
    
    def handle_stop_control_session_srv(self, req):
        global_printer.print_green(f"Received service REQUEST /stop_control_session_srv -> {req.data}")
        self.mission_complete = True
        beep(duration=0.1, freq=750)
        # self.stop_robot_order = True
        return SetBoolResponse(success=True, message="Robot is stopped")
        
    def target_pose_callback(self, msg: PoseStamped):
        """ Xử lý dữ liệu Pose cho mục tiêu """
        if not self.got_first_target_event:
            self.first_goal_get_time = rospy.Time.now().to_sec()
            global_printer.print_blue(f"----------- EVENT: First goal get ----------- {self.first_goal_get_time-self.trigger_time}", background=True)

            self.first_goal_get_robot_pos = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z])
            self.got_first_target_event = True

            if not self.trigger_dummy_run:  # in case a goal is pub directly
                self.robot_init_pos = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z])
        
        self.target_pose = copy.deepcopy(msg)
        if MODIFY_Z_UP_GOAL:
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

    def trigger_pose_callback(self, msg: PoseStamped):
        """ Xử lý dữ liệu Pose cho mục tiêu """
        if not self.trigger_dummy_run:
            object_pose = copy.deepcopy(msg)
            if MODIFY_Z_UP_GOAL:
                # Chuyển đổi vị trí
                object_pose_x = object_pose.pose.position.x
                object_pose_y = -msg.pose.position.z
            else:
                object_pose_x = object_pose.pose.position.x
                object_pose_y = object_pose.pose.position.y
            object_pose_z = msg.pose.position.y
            if object_pose_x >= self.dummy_run_trigger_zone_x[0] and object_pose_x <= self.dummy_run_trigger_zone_x[1] and \
                object_pose_y >= self.dummy_run_trigger_zone_y[0] and object_pose_y <= self.dummy_run_trigger_zone_y[1]:
                self.trigger_time = rospy.Time.now().to_sec()
                global_printer.print_blue(f"----------- EVENT: Trigger ----------- {0}", background=True)

                global_printer.print_green('Trigger dummy run, becareful !')
                self.trigger_dummy_run = True
                self.robot_init_pos = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z])

                self.dummy_run(self.trigger_time, DUMMY_RUN_TIME, DUMMY_RUN_VEL)
                print('TRIGGER POS: ', object_pose_x, object_pose_y, object_pose_z)

                if object_pose_x - self.dummy_run_trigger_zone_x[0] > 0.2:
                    global_printer.print_red('Trigger moment is too late, becareful !')


    def send_udp_message(self, vx, vy, wz):
        if NO_CONTROL:
            return
        # print(f"Sending UDP: {forward_velocity}, {angular_velocity}")
        self.cmd.mode = 2
        self.cmd.gaitType = GAIT_TYPE
        self.cmd.velocity = [vx, vy]
        self.cmd.yawSpeed = wz
        self.cmd.footRaiseHeight = 0.08
        self.cmd.bodyHeight = 0.0
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    def publish_velocity(self, vx, vy, wz):
        if self.no_cmd:
            return
        
        if self.using_real_robot:
            self.send_udp_message(vx, vy, wz)
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

    def dummy_run(self, time_start, time_run, vel_max):
        delta_t = rospy.Time.now().to_sec() - time_start
        if delta_t < time_run:
            if delta_t < 1e-6:
                vx = vel_max
            else:
                vx = vel_max/(delta_t*0.5)
            vx = max(min(vx, vel_max), 0.1)
            # self.send_udp_message(vx, 0.0, 0.0)
            self.publish_velocity(vx, 0.0, 0.0)
            print('Dummy run - time: ', delta_t)
        else:
            pass

    def process_movement(self, last_time):
        # check if in active zone
        # if self.stop_robot_order:
        #     self.mission_complete = True
        #     print('============= STOP because of stop order =============')
        #     self.publish_velocity(0.0, 0.0, 0.0)
        #     return None, None
        
        if  self.robot_pose.pose.position.x < self.active_zone_x[0] or \
            self.robot_pose.pose.position.x > self.active_zone_x[1] or \
            self.robot_pose.pose.position.y < self.active_zone_y[0] or \
            self.robot_pose.pose.position.y > self.active_zone_y[1]:
            global_printer.print_red(f"Robot out of active zone. Robot xy: {self.robot_pose.pose.position.x}, {self.robot_pose.pose.position.y}")
            start_pub_time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start_pub_time < 3.0:
                # self.send_udp_message(0.0, 0.0, 0.0)
                self.publish_velocity(0.0, 0.0, 0.0)
            self.mission_complete = True
            return None, None

        if self.target_pose is None and self.trigger_dummy_run==True:
            self.dummy_run(self.trigger_time, DUMMY_RUN_TIME, DUMMY_RUN_VEL)
            # self.target_pose = PoseStamped()
            # self.target_pose.pose.position.x = 2.0
            # self.target_pose.pose.position.y = -1.0
            # self.target_pose.pose.position.z = 0.1
            # self.target_pose.pose.orientation.z = 1.0
            return None, None
        if self.robot_pose is None or self.target_pose is None:
            # rospy.logwarn("No pose message received.")
            print("No pose message received.")
            return None, None

        # print('\n-----')
        # global_printer.print_yellow(f'GOAL: {self.target_pose.pose.position.x}, {self.target_pose.pose.position.y}')

        delta_t = rospy.Time.now().to_sec() - last_time
        robot_pos = [self.robot_pose.pose.position.x, self.robot_pose.pose.position.y]
        robot_quat = [self.robot_pose.pose.orientation.x, self.robot_pose.pose.orientation.y, self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w]
        goal_pos = [self.target_pose.pose.position.x, self.target_pose.pose.position.y]
        dis_xy = math.sqrt((robot_pos[0] - goal_pos[0])**2 + (robot_pos[1] - goal_pos[1])**2)
        if dis_xy <= CTRL_TOLERANCE_XY:
            self.mission_complete = True
            # self.send_udp_message(0.0, 0.0, 0.0)
            print('============= STOP =============')
            self.publish_velocity(0.0, 0.0, 0.0)
            return None, None

        # print(f'real hz = {1/(rospy.Time.now().to_sec() - last_time):.3f}')
        vx, vy, wz = self.pid.calculate(robot_pos, goal_pos, robot_quat, delta_t)
        # print('forward_velocity: ', forward_velocity)
        # print(f'    vx: {vx}, vy: {vy}, wz: {wz*180/np.pi:.3f}')
        wz = 0
        # self.send_udp_message(vx, vy, wz)
        self.publish_velocity(vx, vy, wz)
        # global_printer.print_yellow(f'Command: [{vx:.6f}, {vy:.6f}] - error: {dis_xy:.6f}')
        # print(f'     GOAL: ', goal_pos)

        print('check vx, vy: ', vx, vy)

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
    
    def distance_p_p(self, p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))
           
    def run(self):
        while not rospy.is_shutdown() and self.robot_pose is None:
            # sleep to wait for robot pose
            print('Waiting for robot pose from topic: ', ROBOT_POSE_Z_UP_TOPIC)
        print('Robot pose received !')


        global_printer.print_blue('===================================================', background=True)
        global_printer.print_blue('ARE YOU READY ? Press Enter to start the mission...', background=True)
        global_printer.print_blue('===================================================', background=True); input(); input()

        print('Mission start !')

        informed_new_run = False
        done_get_first_move = False

        rate = rospy.Rate(RATE)
        count_run = 0
        while not rospy.is_shutdown():
            # A. early return

            # A1. no target
            if not self.trigger_dummy_run and self.target_pose is None:
                rate.sleep()
                continue
            
            # A2. Running time 
            if self.trigger_time is not None and (rospy.Time.now().to_sec() - self.trigger_time > MAX_CONTROL_TIME):
                self.mission_complete = True
                print('============= STOP because taking too much time to reach goal =============')
                send_robot_reached_goal_srv(False)

                # reset all variables
                informed_new_run = False
                done_get_first_move = False
                self.reset_controller()
                continue

            # A3. self.mission_complete is activated
            if self.mission_complete:
                print("\n-------- Mission Complete --------")

                reach_goal_time = rospy.Time.now().to_sec()

                # pub command 0
                for i in range(50):
                    self.publish_velocity(0.0, 0.0, 0.0)
                    rate.sleep()

                # wait awhile until the robot is stable
                rospy.sleep(0.2)

                # get final robot pose
                robot_pos_stop = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z])
                dis_run = self.distance_p_p(self.robot_init_pos[:2], robot_pos_stop[:2])
                ctrl_error = self.distance_p_p([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y], [self.target_pose.pose.position.x, self.target_pose.pose.position.y])
                
                if self.trigger_time is None:   # in case a goal was pub directly
                    self.trigger_time = self.first_goal_get_time
                time_run = reach_goal_time - self.trigger_time

                print(f'    Control error: {ctrl_error}')
                global_printer.print_green(f'    Dis run: {dis_run}')
                print(f'    Real avg vel: {dis_run / (time_run):.6f} m/s')
                # reset all variables
                send_robot_reached_goal_srv(True)
                informed_new_run = False
                done_get_first_move = False
                self.reset_controller()
                continue

            
            # B. Got a command and process
            if not informed_new_run:
                global_printer.print_blue('===================================================', background=True)
                global_printer.print_blue(f'                      NEW RUN START #{count_run}', background=True)
                global_printer.print_blue('===================================================', background=True)
                count_run += 1
                wait_first_move_count = 0
                last_time = rospy.Time.now().to_sec()
                informed_new_run = True

            # detect first robot move
            # if not done_get_first_move and self.distance_p_p([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y], [self.robot_init_pos[0], self.robot_init_pos[1]]) > 0.01:
            if not done_get_first_move:
                init_move_dist = self.distance_p_p([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y], [self.robot_init_pos[0], self.robot_init_pos[1]])
                if init_move_dist > 0.01:
                    self.first_move_time = rospy.Time.now().to_sec()
                    done_get_first_move = True
                    global_printer.print_blue(f"----------- EVENT: First move ----------- {self.first_move_time-self.trigger_time}", background=True)
                    wait_first_move_count = 0
                else:
                    print('waiting for first move... Now dist: ', init_move_dist)
                    wait_first_move_count += 1
                    if wait_first_move_count > 100:
                        warn_beep(5)
                        # shutdown_node()

            # if DEBUG: self.receive_udp_robot_state()
            vx, vy = self.process_movement(last_time=last_time)
            last_time = rospy.Time.now().to_sec()

            rate.sleep()


if __name__ == '__main__':
    try:
        node = RobotController()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted, shutting down.")
