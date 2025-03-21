#! /usr/bin/python3

import rospy
# Importação da biblioteca de mensagens de trajetoria
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('move_arm')

# Criação do publisher no topic /arm_controller/command 
# de um mensagem de tipo JointTrajectory
pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)

cmd = JointTrajectory()
cmd.joint_names.append("arm_1_joint")
cmd.joint_names.append("arm_2_joint")
cmd.joint_names.append("arm_3_joint")
cmd.joint_names.append("arm_4_joint")
cmd.joint_names.append("arm_5_joint")
cmd.joint_names.append("arm_6_joint")
cmd.joint_names.append("arm_7_joint")

point = JointTrajectoryPoint()
point.positions = [0] * 7
point.time_from_start = rospy.Duration(1)

cmd.points.append(point)

rate = rospy.Rate(1)

angle = 0.1

while not rospy.is_shutdown():

    cmd.points[0].positions[1] = angle
    cmd.points[0].time_from_start = rospy.Duration(1)

    pub.publish(cmd)
    angle += 0.1
    rate.sleep()
