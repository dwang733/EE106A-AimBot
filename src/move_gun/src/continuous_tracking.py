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
from datetime import datetime

planner = PathPlanner("right_arm")


def calc_line(trans):
    print("moving arm")
    # x_target = trans.transform.translation.x
    # y_target = trans.transform.translation.y
    # z_target = trans.transform.translation.z
    x_target, y_target, z_target = trans
    print("{},{},{}".format(x_target, y_target, z_target))

    while not rospy.is_shutdown():
        try:
            goal_1 = PoseStamped()
            goal_1.header.frame_id = "base"

            if y_target < 0.25:
                print("target in range")
                # x, y, and z position
                goal_1.pose.position.x = 0.5
                goal_1.pose.position.y = y_target
                goal_1.pose.position.z = z_target
                
                # Orientation as a quaternion (must be normalized to one)
                # q = quaternion_from_euler(2.940, -0.340, -1.65)
                q = quaternion_from_euler(3.000, -0.144, -1.65)
                goal_1.pose.orientation.x = q[0]
                goal_1.pose.orientation.y = q[1]
                goal_1.pose.orientation.z = q[2]
                goal_1.pose.orientation.w = q[3]
            else:
                print("target out of range")
                break
            #     goal_1.pose.position.x = 0.5
            #     goal_1.pose.position.y = 0.15
            #     # goal_1.pose.position.z = 0
            #     goal_1.pose.position.z = z_target
                
            #     # Orientation as a quaternion (must be normalized to one)
            #     theta_yaw = -1.65 + np.arctan((y_target - goal_1.pose.position.y) / (x_target - goal_1.pose.position.x))
            #     # theta_roll = 2.940 + np.arctan((z_'target - goal_1.pose.position.z) / (x_target - goal_1.pose.position.x))
            #     # if theta_roll > np.pi:
            #     #     theta_roll -= 2 * np.pi
            #     theta_roll = 2.940
            #     q = quaternion_from_euler(theta_roll, -0.340, theta_yaw)
            #     goal_1.pose.orientation.x = q[0]
            #     goal_1.pose.orientation.y = q[1]
            #     goal_1.pose.orientation.z = q[2]
            #     goal_1.pose.orientation.w = q[3]

            # Might have to edit this . . . 
            plan = planner.plan_to_pose(goal_1, [])
            if len(plan.joint_trajectory.points) == 0:
                raise Exception("No motion plan found for predicted position")

            # raw_input("Press <Enter> to move the right arm to goal pose 1: ")
            while not planner.execute_plan(plan):
                continue

            # raw_input("Press <Enter> to shoot: ")
            # shoot.shoot()
            # raw_input("Press <Enter> when done reloading: ")
            # shoot.hold()
        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

def main():
    rospy.init_node('move_arm')
    print("init node")

    shoot.init()
    print("init shoot")

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    source_frame = "base"
    target_frame = "target_new"
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        previous_time_stamp = 0
        previous_trans = None
        while True:
            try:
                # trans = tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time(0))
                trans = tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time.now() + rospy.Duration(336.5), rospy.Duration(1))
                unix_timestamp = trans.header.stamp.secs * 10**9 + trans.header.stamp.nsecs
                if unix_timestamp <= previous_time_stamp:
                    continue

                if previous_trans is not None:
                    this_trans = trans.transform.translation
                    x_diff = abs(previous_trans.x - this_trans.x)
                    y_diff = abs(previous_trans.y - this_trans.y)
                    z_diff = abs(previous_trans.z - this_trans.z)

                ERROR_MARGIN = 0.02
                if previous_trans is None or x_diff > ERROR_MARGIN or \
                        y_diff > ERROR_MARGIN or z_diff > ERROR_MARGIN:
                    print(datetime.utcfromtimestamp(trans.header.stamp.secs).strftime('%M:%S'))
                    previous_time_stamp = unix_timestamp
                    trans = trans.transform.translation
                    previous_trans = trans
                    calc_line((trans.x, trans.y, trans.z))
                    print('---------------------')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rate.sleep()
                continue


if __name__ == '__main__':
    main()

