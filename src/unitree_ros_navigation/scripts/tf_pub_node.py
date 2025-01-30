#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf.transformations as tf_trans

MODIFY_Z_UP = True
class PoseToTFPublisher:
    def __init__(self):
        rospy.init_node("pose_to_tf_publisher")

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribe to PoseStamped topics
        self.pose1_sub = rospy.Subscriber("/mocap_pose_topic/dog_pose/", PoseStamped, self.pose1_callback)
        self.pose2_sub = rospy.Subscriber("/mocap_pose_topic/chip_star_pose", PoseStamped, self.pose2_callback)

        rospy.loginfo("Pose to TF node initialized.")

    def publish_transform(self, pose_msg, frame_id, child_frame_id, hardcore_pos_y_up=None):
        """ Convert PoseStamped to TF and publish """
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        if hardcore_pos_y_up is not None:
            t.transform.translation.x = hardcore_pos_y_up[0]
            if MODIFY_Z_UP:
                t.transform.translation.y = -hardcore_pos_y_up[2]
                t.transform.translation.z = hardcore_pos_y_up[1]
                t.transform.rotation.x = 0
                t.transform.rotation.y = 0
                t.transform.rotation.z = 0
                t.transform.rotation.w = 1
            else:
                t.transform.translation.y = hardcore_pos_y_up[1]
                t.transform.translation.z = hardcore_pos_y_up[2]
                t.transform.rotation.x = 0
                t.transform.rotation.y = 0
                t.transform.rotation.z = 0
                t.transform.rotation.w = 1
        else:
            t.transform.translation.x = pose_msg.pose.position.x
            if MODIFY_Z_UP:
                t.transform.translation.y = - pose_msg.pose.position.z
                t.transform.translation.z = pose_msg.pose.position.y
                q_orig = pose_msg.pose.orientation
                q_new = [q_orig.x, -q_orig.z, q_orig.y, q_orig.w]  # Reordering quaternion axes
                # Normalize quaternion (to ensure no numerical instability)
                q_new = tf_trans.unit_vector(q_new)

                # Assign new quaternion
                t.transform.rotation.x = q_new[0]
                t.transform.rotation.y = q_new[1]
                t.transform.rotation.z = q_new[2]
                t.transform.rotation.w = q_new[3]
            
            else:
                t.transform.translation.y = pose_msg.pose.position.y
                t.transform.translation.z = pose_msg.pose.position.z
                t.transform.rotation = pose_msg.pose.orientation
        
        # Publish TF
        self.tf_broadcaster.sendTransform(t)

    def pose1_callback(self, msg):
        print(1)
        self.publish_transform(msg, "world", "dog_frame", hardcore_pos_y_up=[2, 0, 1.5])

    def pose2_callback(self, msg):
        print(2)
        self.publish_transform(msg, "world", "chip_star_frame")

if __name__ == "__main__":
    node = PoseToTFPublisher()
    rospy.spin()
