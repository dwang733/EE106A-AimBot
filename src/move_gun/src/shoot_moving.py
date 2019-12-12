#!/usr/bin/env python

import rospy
import sys
import numpy as np
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Vector3
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from path_planner import PathPlanner
from baxter_interface import Limb
import traceback
import shoot
from extrapolation import ExtrapolationQueue
from datetime import datetime

planner = PathPlanner("right_arm")
RATE_FREQ = 4.0
TIME_OFFSET = 2.5


def create_goal_pose(trans):
    x_target = trans.x
    y_target = trans.y
    z_target = trans.z

    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"

    # if y_target < 0.25:
    #     print("target in range")
    #     # x, y, and z position
    #     goal_1.pose.position.x = 0.5
    #     goal_1.pose.position.y = y_target
    #     goal_1.pose.position.z = z_target
        
    #     # Orientation as a quaternion (must be normalized to one)
    #     # q = quaternion_from_euler(2.940, -0.340, -1.65)
    #     q = quaternion_from_euler(3.000, -0.144, -1.65)
    #     goal_1.pose.orientation.x = q[0]
    #     goal_1.pose.orientation.y = q[1]
    #     goal_1.pose.orientation.z = q[2]
    #     goal_1.pose.orientation.w = q[3]
    # else:
    #     print("target out of range")
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

    return goal_1


def calc_line(trans, vel, extra, START_TIME):
    print("moving arm")
    x_target = trans.x
    y_target = trans.y
    z_target = trans.z
    print("{},{},{}".format(x_target, y_target, z_target))

    try:
        goal_1 = create_goal_pose(trans)
        plan = planner.plan_to_pose(goal_1, [])
        print("initial trans: {}".format(trans))

        if len(plan.joint_trajectory.points) > 0:
            print("num points in traj: {}".format(len(plan.joint_trajectory.points)))
            time_to_execute = plan.joint_trajectory.points[-1].time_from_start.to_sec()
            print(time_to_execute)

            predict_trans = Vector3()
            # predict_trans.x = x_target + vel.x * time_to_execute
            # predict_trans.y = y_target + vel.y * time_to_execute
            # predict_trans.z = z_target + vel.z * time_to_execute
            # print("predict trans: {}".format(predict_trans))
            time = rospy.Time().now() + rospy.Duration(337)
            time = (time.secs + time_to_execute + TIME_OFFSET) * 10**9 + time.nsecs
            predict_trans.x, predict_trans.y, predict_trans.z = extra.extrapolate(time)
            print("{},{},{}".format(predict_trans.x, predict_trans.y, predict_trans.z))

            predict_goal = create_goal_pose(predict_trans)
            plan = planner.plan_to_pose(predict_goal, [])
            print("new plan")
            print("--------------------------------------")
            if len(plan.joint_trajectory.points) == 0:
                raise Exception("No motion plan found for predicted position")

            while not planner.execute_plan(plan):
                continue
            # if not planner.execute_plan(plan):
            #     raise Exception("Execution failed")

            print("shooting!")
            shoot.shoot()
            raw_input("Press <Enter> when done reloading: ")
            shoot.hold()

        print("Program done!")
        exit(0)
    except Exception as e:
        print(e)
        traceback.print_exc()
        exit(0)

def main():
    rospy.init_node('move_arm')
    print("init node")

    shoot.init()
    print("init shoot")

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    source_frame = "base"
    # target_frame = "target"
    target_frame = "target_new"

    previous_trans = None
    previous_time_stamp = None
    vel = None

    extra = ExtrapolationQueue(6, deg=2)

    raw_input("Press <Enter> to move the right arm: ")
    while not rospy.is_shutdown():
        START_TIME = None
        rate = rospy.Rate(RATE_FREQ)
        previous_time_stamp = 0
        start_loop_time = rospy.Time().now()
        while True:
            try:
                trans = tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time(0))
                # if previous_trans is None:
                #     # print("no previous trans")
                #     previous_trans = trans.transform.translation
                #     previous_time_stamp = trans.header.stamp
                #     rate.sleep()
                #     continue

                # time_stamp = trans.header.stamp
                # trans = trans.transform.translation

                # dur = (time_stamp - previous_time_stamp).to_sec()
                # print(dur)
                # current_vel = Vector3()
                # current_vel.x = (trans.x - previous_trans.x) / dur
                # current_vel.y = (trans.y - previous_trans.y) / dur
                # current_vel.z = (trans.z - previous_trans.z) / dur
                # if abs(current_vel.x) < 0.05 and abs(current_vel.y) < 0.05 and abs(current_vel.z) < 0.05:
                #     print("velocity too small")
                #     # print("velocity: {},{},{}".format(current_vel.x, current_vel.y, current_vel.z))
                # else:
                #     print("velocity: {},{},{}".format(current_vel.x, current_vel.y, current_vel.z))

                # previous_trans = trans
                # previous_time_stamp = time_stamp
                # if vel is None:
                #     vel = current_vel
                # else:
                #     # vel = 0.5 * vel + 0.5 * current_vel
                #     vel = current_vel
                # rate.sleep()

                # extra.push(trans.header.stamp.secs, trans.transform.translation)
                # if START_TIME is None:
                #     START_TIME = trans.header.stamp.secs
                # rate.sleep()

                unix_timestamp = trans.header.stamp.secs * 10**9 + trans.header.stamp.nsecs
                if unix_timestamp > previous_time_stamp:
                    print(trans.header.stamp.nsecs / 10**6)
                    print(trans.transform.translation)
                    print("got new transform")
                    extra.push(unix_timestamp, trans.transform.translation)
                    previous_time_stamp = unix_timestamp
                    if START_TIME is None:
                        START_TIME = trans.header.stamp.secs

                if (rospy.Time.now() - start_loop_time).to_sec() > 0.5:
                    print("done getting transforms")
                    break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print("got exception")
                rate.sleep()

        # calc_line(trans, vel)
        calc_line(trans.transform.translation, 0, extra, START_TIME)
        print('end of while loop')


if __name__ == '__main__':
    main()

