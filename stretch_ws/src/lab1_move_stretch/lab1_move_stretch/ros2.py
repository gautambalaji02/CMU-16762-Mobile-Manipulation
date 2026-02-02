#!/usr/bin/env python3

import numpy as np
import hello_helpers.hello_misc as hm
import time


class StretchDemoNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        # Initialize the HelloNode
        hm.HelloNode.main(self, 'stretch_demo_node', 'stretch_demo_node', wait_for_first_pointcloud=False)
        
        print('Robot connected via ROS2\n')

        # Stow the robot
        print('Stowing the robot...\n')
        self.stow_the_robot()
        time.sleep(2.0)
        print('Robot stowed!\n')

        # Get the current joint states to determine max limits
        # For Stretch 3: arm range is ~0.52m, lift range is ~1.1m
        arm_max = 0.52
        lift_max = 1.1

        # Extend the telescoping arm all the way out and raise the lift all the way up
        print('Extending arm and raising lift...\n')
        self.move_to_pose({
            'joint_arm': arm_max - 0.1,
            'joint_lift': lift_max - 0.1
        }, blocking=True)
        print('Arm extended and lift raised!\n')

        # Move wrist motors one at a time
        print('Setting wrist motors individually...\n')

        print('Setting wrist_yaw to 45 degrees...')
        self.move_to_pose({'joint_wrist_yaw': np.deg2rad(45)}, blocking=True)
        
        print('Setting wrist_pitch to -30 degrees...')
        self.move_to_pose({'joint_wrist_pitch': np.deg2rad(-30)}, blocking=True)
        
        print('Setting wrist_roll to 90 degrees...')
        self.move_to_pose({'joint_wrist_roll': np.deg2rad(90)}, blocking=True)

        # Open and close the gripper
        # Gripper aperture: positive values open, negative values close
        # Range is typically -0.3 to 0.6 radians (varies by gripper)
        print('Opening gripper to 100%...\n')
        self.move_to_pose({'joint_gripper_finger_left': 0.5}, blocking=True)
        time.sleep(1.0)

        print('Closing gripper to 0%...\n')
        self.move_to_pose({'joint_gripper_finger_left': -0.2}, blocking=True)
        time.sleep(1.0)

        # Rotate head pan and tilt
        print('Rotating head pan and tilt...\n')
        self.move_to_pose({
            'joint_head_pan': np.deg2rad(45),
            'joint_head_tilt': np.deg2rad(-30)
        }, blocking=True)

        # Bring robot back to stowed position
        print('Bringing robot back to stowed position...\n')
        self.stow_the_robot()
        time.sleep(2.0)
        print('Robot stowed!\n')

        # Drive the robot forward by 0.5 meters
        print('Driving robot forward by 0.5 meters...\n')
        self.move_to_pose({'translate_mobile_base': 0.5}, blocking=True)

        # Turn the base around by 180 degrees
        print('Turning robot around by 180 degrees...\n')
        self.move_to_pose({'rotate_mobile_base': np.pi}, blocking=True)

        # Drive robot back to original position
        print('Driving robot back to original position...\n')
        self.move_to_pose({'translate_mobile_base': 0.5}, blocking=True)

        print('Demo complete!\n')


if __name__ == '__main__':
    try:
        node = StretchDemoNode()
        node.main()
    except KeyboardInterrupt:
        print('\nRobot demo interrupted by user.')
