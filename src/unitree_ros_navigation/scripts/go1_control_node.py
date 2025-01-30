#!/usr/bin/env python3
#original shebang ####### !/home/server-huynn/anaconda3/envs/nae-dynamic-3/bin/python
# Laat de robot bewegen naar een doel met dynamische yaw-correctie.
# Desired yaw wordt berekend in het lokale frame van de robot en is daarom gelijk aan de yaw difference tussen de robot en het doel.
# Dus eigenlijk is desired yaw gelijk aan de yaw_rate
import rospy
import math
import tf
import sys
sys.path.append('/home/server-huynn/workspace/robot_catching_project/trajectory_prediction/go1-control-rocat/unitree_go1_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/lib/python/amd64')
import robot_interface as sdk
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import time

from python_utils import printer

global_printer = printer.Printer()


# Configuratie
HIGHLEVEL = 0xee
robot_ip = "192.168.123.161"  # Robot IP
udp = sdk.UDP(HIGHLEVEL, 8080, robot_ip, 8082)
# Initialiseer commando's
cmd = sdk.HighCmd()
udp.InitCmdData(cmd)
# Globale variabelen
tf_listener = None
mission_complete = False
distance_threshold = 0.4  # Stopafstand (meters)
yaw_threshold = math.radians(20)  #4 graden in radialen
RATE = 20  # Tijdstap (seconden)
ROBOT_TF_FRAME = "dog_frame"
TARGET_TF_FRAME = 'pred_impact_point_frame' # 'pred_impact_point_frame' # "chip_star_frame"



def shutdown_node():
    rospy.loginfo("Shutting down the node...")
    rospy.signal_shutdown("User requested shutdown")


def publish_velocity(vx, wz):
    # Tạo publisher trên topic "/cmd_vel"
    velocity_pub = rospy.Publisher("/check/cmd_vel", Twist, queue_size=10)
    vel_msg = Twist()
    # Thiết lập vận tốc tuyến tính (m/s)
    vel_msg.linear.x = vx  # Tiến về phía trước
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    # Thiết lập vận tốc góc (rad/s)
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = wz  # Xoay trái
    # Publish message
    velocity_pub.publish(vel_msg)

def calculate_relative_position(source_frame, target_frame):
    """
    Bereken de relatieve positie en yaw-afwijking tussen source_frame en target_frame.
    """
    try:
        now = rospy.Time(0)
        tf_listener.waitForTransform(source_frame, target_frame, now, rospy.Duration(1000.0))
        (trans, rot) = tf_listener.lookupTransform(source_frame, target_frame, now)
        # Relatieve positie
        dx = trans[0]
        dy = trans[1]
        distance = math.sqrt(dx**2 + dy**2)
        # Bereken desired_yaw in het lokale frame van de robot
        desired_yaw = math.atan2(dy, dx)
        rospy.loginfo(f"Distance to target: {distance:.2f} m, Desired yaw: {math.degrees(desired_yaw):.2f}°")
        return distance, desired_yaw
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn(f"TF Exception: {str(e)}")
        return None, None
def send_udp_message(forward_velocity, yaw_rate):
    """
    Stuur een UDP-bericht naar de robot.
    """
    print(f"         Sending UDP message {forward_velocity}, {yaw_rate}")
    cmd.mode = 2  # Mode 2: Velocity control
    cmd.gaitType = 1  # Trot gait
    cmd.velocity = [forward_velocity, 0.0]  # Forward velocity
    cmd.yawSpeed = yaw_rate  # Yaw snelheid
    cmd.footRaiseHeight = 0.08
    cmd.bodyHeight = 0.0
    udp.SetSend(cmd)
    udp.Send()

def get_robot_state():
    """
    Lấy trạng thái hiện tại của robot Unitree Go1.
    """
    state = state = sdk.LowState()  # Khởi tạo biến lưu trạng thái
    udp.GetRecv(state)  # Truy xuất trạng thái từ UDP
    if state is not None:
        print(f"Robot LowState: {state}")
        return state
    else:
        print("No response from robot!")
        return None


def lay_down_robot():
    """
    Leg de robot neer (veiligheidsstop).
    """
    rospy.loginfo("Laying down the robot...")
    cmd.mode = 5  # Mode 5: Position stand down
    cmd.gaitType = 0
    cmd.velocity = [0.0, 0.0]
    cmd.yawSpeed = 0.0
    cmd.footRaiseHeight = 0.0
    cmd.bodyHeight = -0.2
    udp.SetSend(cmd)
    udp.Send()
def process_movement():
    """
    Verwerk de beweging naar het target met dynamische yaw-correctie.
    """
    global mission_complete
    if mission_complete:
        return
    # Bereken de relatieve positie en yaw-afwijking
    distance, desired_yaw = calculate_relative_position(ROBOT_TF_FRAME, TARGET_TF_FRAME)
    if distance is None or desired_yaw is None:
        return
    # Controleer of het doel bereikt is
    if distance < distance_threshold:
        rospy.loginfo("Target bereikt. Stopping robot.")
        mission_complete = True
        send_udp_message(0.0, 0.0)
        lay_down_robot()
        return
    # Corrigeer yaw en bepaal forward velocity
    if abs(desired_yaw) > yaw_threshold:
        rospy.loginfo(f"Yaw correction: {math.degrees(desired_yaw):.2f}°")
        forward_velocity = 0.0  # Stop forward movement tijdens yaw-correctie
        yaw_rate = max(min(desired_yaw, 0.5), -0.5)  # Limiteer yaw rate binnen [-0.5, 0.5]
    else:
        forward_velocity = max(min(0.4 - abs(desired_yaw) * 0.8, 0.4), 0.1)*6
        yaw_rate = max(min(desired_yaw, 0.5), -0.5)*2
    # Stuur de commando's naar de robot
    send_udp_message(forward_velocity, yaw_rate)
    publish_velocity(forward_velocity, yaw_rate)
    get_robot_state()

def listener():
    """
    ROS-node initialisatie.
    """
    global tf_listener
    rospy.init_node('dynamic_yaw_correction', anonymous=True)
    rospy.loginfo("Dynamic Yaw Correction Node gestart.")
    # Initialiseer de TF Listener
    tf_listener = tf.TransformListener()
    # Verwerk beweging in een loop
    rate = rospy.Rate(RATE)  # Frequentie gebaseerd op de tijdstap
    
    time_check_rate_start = time.time()
    time_start = time.time()
    while not rospy.is_shutdown():
        rate_check = 1/(time.time() - time_check_rate_start)
        global_printer.print_green(f"Control rate: {rate_check}")
        time_check_rate_start = time.time()
        process_movement()

        if mission_complete:
            print('\n\n\n------------------------------------')
            print(f"Time run: {time.time() - time_start}")
            print('------------------------------------')
            shutdown_node()
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted, shutting down.")