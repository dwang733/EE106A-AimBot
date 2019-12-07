#!/usr/bin/env python

"""
Script to shoot the Nerf gun based on keyboard inputs
"""

import argparse

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

def shoot():
    gripper = baxter_interface.Gripper('right', CHECK_VERSION)

    if gripper.type() != 'electric':
        capability_warning(gripper, 'command_position')
        return
    gripper.command_position(70)
    rospy.sleep(0.025)
    gripper.close()
    rospy.sleep(1)
    reload()


def reload():
    gripper = baxter_interface.Gripper('right', CHECK_VERSION)
    if gripper.type() != 'electric':
        capability_warning(gripper, 'command_position')
        return
    gripper.command_position(70)


def map_keyboard():
    # initialize interfaces
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    left = baxter_interface.Gripper('left', CHECK_VERSION)
    right = baxter_interface.Gripper('right', CHECK_VERSION)

    # Set holding force and velocity to max
    left.set_holding_force(100.0)
    right.set_holding_force(100.0)
    left.set_velocity(100.0)
    right.set_velocity(100.0)


    def clean_shutdown():
        if not init_state:
            print("Disabling robot...")
            rs.disable()
        print("Exiting example.")
    rospy.on_shutdown(clean_shutdown)

    def capability_warning(gripper, cmd):
        msg = ("%s %s - not capable of '%s' command" %
               (gripper.name, gripper.type(), cmd))
        rospy.logwarn(msg)

    def offset_position(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'command_position')
            return
        current = gripper.position()
        print(current)
        gripper.command_position(current + offset)
        print(current + offset)
        print(gripper.position())
        print("---------")

    def reload(gripper):
    	if gripper.type() != 'electric':
            capability_warning(gripper, 'command_position')
            return
        # gripper.command_position(52)
        gripper.command_position(60)

    def shoot(gripper):
    	if gripper.type() != 'electric':
            capability_warning(gripper, 'command_position')
            return
        # gripper.command_position(52)
        gripper.command_position(60)
        rospy.sleep(0.025)
        gripper.close()
        rospy.sleep(1)
        reload(gripper)

    def hold(gripper):
    	if gripper.type() != 'electric':
            capability_warning(gripper, 'command_position')
            return
        gripper.command_position(34)

    bindings = {
    #   key: (function, args, description)
        'r': (left.reboot, [], "left: reboot"),
        'R': (right.reboot, [], "right: reboot"),
        'c': (left.calibrate, [], "left: calibrate"),
        'C': (right.calibrate, [], "right: calibrate"),
        'q': (left.close, [], "left: close"),
        'Q': (right.close, [], "right: close"),
        'w': (left.open, [], "left: open"),
        'W': (right.open, [], "right: open"),
        's': (left.stop, [], "left: stop"),
        'S': (right.stop, [], "right: stop"),
        'u': (offset_position, [left, -10.0], "left:  decrease position"),
        'U': (offset_position, [right, -10.0], "right:  decrease position"),
        'i': (offset_position, [left, 10.0], "left:  increase position"),
        'I': (offset_position, [right, 10.0], "right:  increase position"),
        'f': (reload, [left], "left: reload"),
        'F': (reload, [right], "right: reload"),
        'g': (hold, [left], "left: hold"),
        'G': (hold, [right], "right: hold"),
        'h': (shoot, [left], "left: shoot"),
        'H': (shoot, [right], "right: shoot"),
    }

    done = False
    print("Enabling robot... ")
    rs.enable()
    print("Controlling grippers. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            if c in ['\x1b', '\x03']:
                done = True
            elif c in bindings:
                cmd = bindings[c]
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))
    # force shutdown call if caught by key handler
    rospy.signal_shutdown("Example finished.")



def main():
    """RSDK Gripper Example: Keyboard Control

    Use your dev machine's keyboard to control and configure
    Baxter's grippers.

    Run this example to command various gripper movements while
    adjusting gripper parameters, including calibration, velocity,
    and force. Uses the baxter_interface.Gripper class and the
    helper function, baxter_external_devices.getch.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("shoot_keyboard")

    map_keyboard()


if __name__ == '__main__':
    main()
