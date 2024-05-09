#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import numpy as np


def talker():
    rospy.loginfo('starting the talker node')
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    

    pub= rospy.Publisher('/robot_arm/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
    
    command = FollowJointTrajectoryActionGoal()
    command.header.stamp = rospy.Time.now()
    command.goal.trajectory.joint_names = ['arm_base_to_base', 'arm_joint_01', 'arm_joint_02', 'wrist_joint_01', 'wrist_joint_04']
    point_1 = JointTrajectoryPoint()
    point_1.positions=[0, 0, 0, 0, 0]
    point_1.velocities=[0, 0, 1.5, 0, 0] 
    point_1.time_from_start = rospy.Duration(5)
    command.goal.trajectory.points.append(point_1)
    
    point_2 = JointTrajectoryPoint()
    point_2.positions=[0, 0, math.pi/2, 0, 0]
    point_2.velocities=[0, 0, 1.5, 0, 0] 
    point_2.time_from_start = rospy.Duration(10)
    command.goal.trajectory.points.append(point_2)


    pub.publish(command)
    rospy.loginfo('=====send command %r', command.goal.trajectory)

    rate.sleep()

	

if __name__ == '__main__':
    try:
        talker()

    except rospy.ROSInterruptException:
        pass