import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def move_arm():
    pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
    rospy.init_node("arm_mover")

    joint_names = ["arm_base_to_base" ,\
                   "arm_joint_01", \
                   "arm_joint_02", \
                   "wrist_joint_01", \
                   "wrist_joint_04"]

    # Replace with desired target positions for each joint
    target_positions_1, t_1 = [1.0, 0.0, 0.0, 0.0, 0.0], 1.0
    target_positions_2, t_2 = [1.0, 1.0, 0.0, 0.0, 0.0], 2.0
    target_positions_3, t_3 = [1.0, 1.0, 1.0, 0.0, 0.0], 3.0

    rate = rospy.Rate(10)  # 10 Hz

    # Create a trajectory message
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    # Define a single point trajectory
    point_1 = JointTrajectoryPoint()
    point_1.positions = target_positions_1
    point_1.velocities = []
    point_1.accelerations = []
    point_1.effort = []
    point_1.time_from_start = rospy.Duration(t_1)

    point_2 = JointTrajectoryPoint()
    point_2.positions = target_positions_2
    point_2.velocities = []
    point_2.accelerations = []
    point_2.effort = []
    point_2.time_from_start = rospy.Duration(t_2)

    point_3 = JointTrajectoryPoint()
    point_3.positions = target_positions_3
    point_3.velocities = []
    point_3.accelerations = []
    point_3.effort = []
    point_3.time_from_start = rospy.Duration(t_3)

    trajectory.points.append(point_1)
    trajectory.points.append(point_2)
    trajectory.points.append(point_3)

    while not rospy.is_shutdown():
        # Publish the trajectory
        pub.publish(trajectory)
        rate.sleep()


if __name__ == "__main__":
    move_arm()
    rospy.spin()