#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

Joints = 6
jnt_pos_start = np.zeros(Joints)

def get_shoulder_pan_joint_position(ctr_msg):
    global jnt_pos_start
    jnt_pos_start[0] = ctr_msg.process_value

def get_shoulder_lift_joint_position(ctr_msg):
    global jnt_pos_start
    jnt_pos_start[1] = ctr_msg.process_value

def get_elbow_joint_position(ctr_msg):
    global jnt_pos_start
    jnt_pos_start[2] = ctr_msg.process_value

def get_wrist_1_joint_position(ctr_msg):
    global jnt_pos_start
    jnt_pos_start[3] = ctr_msg.process_value

def get_wrist_2_joint_position(ctr_msg):
    global jnt_pos_start
    jnt_pos_start[4] = ctr_msg.process_value

def get_wrist_3_joint_position(ctr_msg):
    global jnt_pos_start
    jnt_pos_start[5] = ctr_msg.process_value

def compute_linear(q_start, q_goal, t, t_max):
    return (q_goal - q_start) * (t / t_max) + q_start

def main():
    rospy.init_node('tcp_control')
    rate = rospy.Rate(100)

    rospy.Subscriber("/shoulder_pan_joint_position_controller/state", JointControllerState, get_shoulder_pan_joint_position)
    rospy.Subscriber("/shoulder_lift_joint_position_controller/state", JointControllerState, get_shoulder_lift_joint_position)
    rospy.Subscriber("/elbow_joint_position_controller/state", JointControllerState, get_elbow_joint_position)
    rospy.Subscriber("/wrist_1_joint_position_controller/state", JointControllerState, get_wrist_1_joint_position)
    rospy.Subscriber("/wrist_2_joint_position_controller/state", JointControllerState, get_wrist_2_joint_position)
    rospy.Subscriber("/wrist_3_joint_position_controller/state", JointControllerState, get_wrist_3_joint_position)

    joint_com_pub = [rospy.Publisher("/shoulder_pan_joint_position_controller/command", Float64, queue_size=1000),
                     rospy.Publisher("/shoulder_lift_joint_position_controller/command", Float64, queue_size=1000),
                     rospy.Publisher("/elbow_joint_position_controller/command", Float64, queue_size=1000),
                     rospy.Publisher("/wrist_1_joint_position_controller/command", Float64, queue_size=1000),
                     rospy.Publisher("/wrist_2_joint_position_controller/command", Float64, queue_size=1000),
                     rospy.Publisher("/wrist_3_joint_position_controller/command", Float64, queue_size=1000)]


    while not rospy.is_shutdown():
        positions = [Float64() for _ in range(Joints)]
        for i in range(Joints):
            positions[i].data = -1.0
            joint_com_pub[i].publish(positions[i])

        rospy.sleep(2)

if __name__ == '__main__':
    main()
