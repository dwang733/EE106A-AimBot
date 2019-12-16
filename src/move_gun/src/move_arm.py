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
                goal_1.pose.position.x = 0.5
                goal_1.pose.position.y = 0.15
                # goal_1.pose.position.z = 0
                goal_1.pose.position.z = z_target
                
                # Orientation as a quaternion (must be normalized to one)
                theta_yaw = -1.65 + np.arctan((y_target - goal_1.pose.position.y) / (x_target - goal_1.pose.position.x))
                # theta_roll = 2.940 + np.arctan((z_'target - goal_1.pose.position.z) / (x_target - goal_1.pose.position.x))
                # if theta_roll > np.pi:
                #     theta_roll -= 2 * np.pi
                theta_roll = 2.940
                q = quaternion_from_euler(theta_roll, -0.340, theta_yaw)
                goal_1.pose.orientation.x = q[0]
                goal_1.pose.orientation.y = q[1]
                goal_1.pose.orientation.z = q[2]
                goal_1.pose.orientation.w = q[3]

            # Might have to edit this . . . 
            plan = planner.plan_to_pose(goal_1, [])
            if len(plan.joint_trajectory.points) == 0:
                raise Exception("No motion plan found for predicted position")

            # raw_input("Press <Enter> to move the right arm to goal pose 1: ")
            while not planner.execute_plan(plan):
                continue

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
    print("init node")

    shoot.init()
    print("init shoot")

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    source_frame = "base"
    target_frame = "target_new"
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        prev_pos = None
        try:
            trans_list = []
            for _ in range(10):
                trans = tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time(0))
                # print(trans.header.stamp)
                print(datetime.utcfromtimestamp(trans.header.stamp.secs).strftime('%M:%S'))
                trans_list.append(trans.transform.translation)
            x = np.median([i.x for i in trans_list])
            y = np.median([i.y for i in trans_list])
            z = np.median([i.z for i in trans_list])
            calc_line((x, y, z))

            # if prev_pos is not None:
            #     diff = np.sqrt((prev_pos[0] - x)**2 + (prev_pos[1] - y)**2 + (prev_pos[2] - z)**2)
            # if prev_pos is None or diff > 5:
            #     calc_line((x, y, z))
            #     prev_pos = (x, y, z)
            # print("----------------------")
            # rate.sleep()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rate.sleep()
            continue


if __name__ == '__main__':
    main()

