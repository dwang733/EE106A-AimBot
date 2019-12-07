#!/usr/bin/env python


import rospy
import sys
import numpy as np
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from path_planner import PathPlanner
from baxter_interface import Limb
import traceback


planner = PathPlanner("right_arm")


def calc_line(trans):
	print("moving arm")
	x_target = trans.transform.translation.x
	y_target = trans.transform.translation.y
	z_target = trans.transform.translation.z

	x_center = 0.5
	y_center = -0.5
	z_center = -0.1

	norm = np.sqrt((x_target - x_center)**2 + (y_target - y_center)**2 + (z_target - z_center)**2)

	# orien_const = OrientationConstraint()
	# orien_const.link_name = "right_gripper"
	# orien_const.header.frame_id = "base"
	# orien_const.orientation.x = (x_target - x_center)/norm
	# orien_const.orientation.y = (y_target - y_center)/norm
	# orien_const.orientation.z = (z_target - z_center)/norm
	# orien_const.absolute_x_axis_tolerance = 0.1
	# orien_const.absolute_y_axis_tolerance = 0.1
	# orien_const.absolute_z_axis_tolerance = 0.1
	# orien_const.weight = 1.0

	while not rospy.is_shutdown():
	    try:
	        goal_1 = PoseStamped()
	        goal_1.header.frame_id = "base"
	        
	        #x, y, and z position
	        goal_1.pose.position.x = x_center
	        goal_1.pose.position.y = y_center
	        goal_1.pose.position.z = z_center
	        
	        #Orientation as a quaternion (must be normalized to one)
	        # goal_1.pose.orientation.x = (x_target - x_center)/norm
	        # goal_1.pose.orientation.y = (y_target - y_center)/norm
	        # goal_1.pose.orientation.z = (z_target - z_center)/norm
	        # goal_1.pose.orientation.w = 0.0
	        q = quaternion_from_euler(-3.14, 0, -1.57)
	        goal_1.pose.orientation.x = q[0]
	        goal_1.pose.orientation.y = q[1]
	        goal_1.pose.orientation.z = q[2]
	        goal_1.pose.orientation.w = q[3]

	        # Might have to edit this . . . 
	        plan = planner.plan_to_pose(goal_1, [])

	        raw_input("Press <Enter> to move the right arm to goal pose 1: ")
	        if not planner.execute_plan(plan):
	            raise Exception("Execution failed")
	    except Exception as e:
	        print e
	        traceback.print_exc()
	    else:
	        break

def main():

	rospy.init_node('move_arm')
	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)

	source_frame = "base"
	target_frame = "target"

	rate = rospy.Rate(1000.0)
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
			calc_line(trans)
			print("getting target location")
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			print(e)
			rate.sleep()
			continue


if __name__ == '__main__':
    main()

