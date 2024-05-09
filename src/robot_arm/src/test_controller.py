#!/usr/bin/env python

import sys, yaml
import rospy, rosbag
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import math
  
def get_cur_pos():
  try:
    value = rospy.wait_for_message("/robot_arm/state", JointTrajectoryControllerState, 5.0)
    state = JointState()
    state.position = value.actual.positions
    return state
  except (rospy.ROSException, rospy.ROSInterruptException):
    rospy.logerr('Unable to read current position')
    raise

def wait_for_subs(pub, num_subs, min_time, timeout):
  end = rospy.Time.now() + rospy.Duration(timeout)
  rospy.sleep(min_time)

  r = rospy.Rate(10)  # check at 10Hz
  while (pub.get_num_connections() < num_subs) and (rospy.Time.now() < end) and not rospy.is_shutdown():
    r.sleep()

  return (pub.get_num_connections() >= num_subs)

def get_joint_names():

  default_names = [
        "arm_base_to_base",
        "arm_joint_01",
        "arm_joint_02",
        "wrist_joint_01",
        "wrist_joint_04",
  ]

  return default_names

def test_move_robot():
  joint_names = get_joint_names()
  
  start = get_cur_pos()
  start_pt = JointTrajectoryPoint()
  start_pt.positions = [0, 0, 0, 0, 0]
  start_pt.velocities = [0]*len(start.position)
  start_pt.time_from_start = rospy.Duration(0.0)

  point_1 = JointTrajectoryPoint()
  point_1.positions = [math.pi / 4, 1, 0, 0, 0]
  point_1.velocities = [0, 0, 0, 0, 0]
  point_1.time_from_start = rospy.Duration(3.0)

  point_2 = JointTrajectoryPoint()
  point_2.positions = [math.pi / 4, 1, 1, 0, 0]
  point_2.velocities = [0, 0, 0, 0, 0]
  point_2.time_from_start = rospy.Duration(6.0)

  point_3 = JointTrajectoryPoint()
  point_3.positions = [0, 0, 0, 0, 0]
  point_3.velocities = [0, 0, 0, 0, 0]
  point_3.time_from_start = rospy.Duration(9.0)

  traj =  JointTrajectory(joint_names=joint_names, points=[start_pt, point_1, point_2, point_3])
  pub = rospy.Publisher('/robot_arm/command', JointTrajectory, queue_size=10)
  if not wait_for_subs(pub, 1, 0.5, 2.0):
    rospy.logwarn('Timeout while waiting for subscribers.  Publishing trajectory anyway.')

  pub.publish(traj)

def main():
  rospy.init_node('move_to_joint')

  try:
    test_move_robot()
  except:
    rospy.logerr('Unable to move to commanded position. Aborting.')
    raise  

if __name__ == "__main__":
  main()
