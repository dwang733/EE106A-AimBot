#!/usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

# RELOAD_POS = 70
# HOLD_POS = 60
RELOAD_POS = 60
HOLD_POS = 45
gripper = None


def init():
    global gripper
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    gripper = baxter_interface.Gripper('right', CHECK_VERSION)
    if gripper.type() != 'electric':
        capability_warning(gripper, 'command_position')
        return
    gripper.set_holding_force(100.0)
    gripper.set_velocity(100.0)
    hold()


def shoot():
    gripper.command_position(RELOAD_POS)
    rospy.sleep(0.025)
    gripper.close()
    rospy.sleep(1)
    reload()


def reload():
    gripper.command_position(RELOAD_POS)


def hold():
    gripper.command_position(HOLD_POS)
