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
import shoot

planner = PathPlanner("right_arm")
print("got path planner")

def calc_line(trans):
    print("moving arm")
    x_target = trans.transform.translation.x
    y_target = trans.transform.translation.y
    z_target = trans.transform.translation.z
    print("{},{},{}".format(x_target, y_target, z_target))

    while not rospy.is_shutdown():
        try:
            goal_1 = PoseStamped()
            goal_1.header.frame_id = "base"
            
            # x, y, and z position
            goal_1.pose.position.x = 0.5
            goal_1.pose.position.y = y_target
            goal_1.pose.position.z = z_target
            
            # Orientation as a quaternion (must be normalized to one)
            q = quaternion_from_euler(3.100, 0.154, -1.428)
            goal_1.pose.orientation.x = q[0]
            goal_1.pose.orientation.y = q[1]
            goal_1.pose.orientation.z = q[2]
            goal_1.pose.orientation.w = q[3]

            # Might have to edit this . . . 
            plan = planner.plan_to_pose(goal_1, [])

            raw_input("Press <Enter> to move the right arm to goal pose 1: ")
            if not planner.execute_plan(plan):
                raise Exception("Execution failed")

            raw_input("Press <Enter> to shoot: ")
            shoot.shoot()
            raw_input("Press <Enter> when done reloading: ")
            shoot.hold()
        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

def main():
    rospy.init_node('move_arm')

    shoot.init()

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    source_frame = "base"
    # target_frame = "target"
    target_frame = "target_new"
    rate = rospy.Rate(1000.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time())
            calc_line(trans)
            print("getting target location")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rate.sleep()
            continue


if __name__ == '__main__':
    main()

