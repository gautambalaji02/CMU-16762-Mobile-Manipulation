import ikpy.urdf.utils
import urchin as urdfpy
import numpy as np
import ikpy.chain
import stretch_body.robot
import importlib.resources as importlib_resources
import time
import hello_helpers.hello_misc as hm
import sys

# Let us try to grab an object and then place it someplace else. 
# NOTE before running: `python3 -m pip install --upgrade ikpy graphviz urchin networkx`

first_target_point = [1.0, 0.0, 0.15]
first_target_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0) # [roll, pitch, yaw]

print("Our first target point: ", first_target_point)
second_target_point = [0.5, 0.5, 0.15]
second_target_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, np.pi/2) # [roll, pitch, yaw]
print("Our second target point: ", second_target_point)

third_target_point = [0.5, -0.5, 0.2]
third_target_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, -np.pi/2) # [roll, pitch, yaw]
print("Our third target point: ", third_target_point)

# # Setup the Python API
# robot = stretch_body.robot.Robot()
# TODO:
# robot.startup()
# # Ensure robot is homed
# if not robot.is_calibrated():
#     robot.home()

def main():
    robot = hm.HelloNode.quick_create('robot')
    
    print("Starting robot...")
    robot.move_to_pose({'joint_lift': 0.5}, blocking=True)
    robot.stow_the_robot()
    # sys.exit(0)

    pkg_path = str(importlib_resources.files('stretch_urdf'))
    urdf_file_path = pkg_path + '/SE3/stretch_description_SE3_eoa_wrist_dw3_tool_sg3.urdf'

    # Remove unnecessary links/joints
    original_urdf = urdfpy.URDF.load(urdf_file_path)
    modified_urdf = original_urdf.copy()

    names_of_links_to_remove = ['link_right_wheel', 'link_left_wheel', 'caster_link', 'link_head', 'link_head_pan', 'link_head_tilt', 'link_aruco_right_base', 'link_aruco_left_base', 'link_aruco_shoulder', 'link_aruco_top_wrist', 'link_aruco_inner_wrist', 'camera_bottom_screw_frame', 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame', 'camera_infra1_frame', 'camera_infra1_optical_frame', 'camera_infra2_frame', 'camera_infra2_optical_frame', 'camera_color_frame', 'camera_color_optical_frame', 'camera_accel_frame', 'camera_accel_optical_frame', 'camera_gyro_frame', 'camera_gyro_optical_frame', 'gripper_camera_bottom_screw_frame', 'gripper_camera_link', 'gripper_camera_depth_frame', 'gripper_camera_depth_optical_frame', 'gripper_camera_infra1_frame', 'gripper_camera_infra1_optical_frame', 'gripper_camera_infra2_frame', 'gripper_camera_infra2_optical_frame', 'gripper_camera_color_frame', 'gripper_camera_color_optical_frame', 'laser', 'base_imu', 'respeaker_base', 'link_wrist_quick_connect', 'link_gripper_finger_right', 'link_gripper_fingertip_right', 'link_aruco_fingertip_right', 'link_gripper_finger_left', 'link_gripper_fingertip_left', 'link_aruco_fingertip_left', 'link_aruco_d405', 'link_head_nav_cam']

    # links_kept = ['base_link', 'link_mast', 'link_lift', 'link_arm_l4', 'link_arm_l3', 'link_arm_l2', 'link_arm_l1', 'link_arm_l0', 'link_wrist_yaw', 'link_wrist_yaw_bottom', 'link_wrist_pitch', 'link_wrist_roll', 'link_gripper_s3_body', 'link_grasp_center']
    links_to_remove = [l for l in modified_urdf._links if l.name in names_of_links_to_remove]
    for lr in links_to_remove:
        modified_urdf._links.remove(lr)
        
    names_of_joints_to_remove = ['joint_right_wheel', 'joint_left_wheel', 'caster_joint', 'joint_head', 'joint_head_pan', 'joint_head_tilt', 'joint_aruco_right_base', 'joint_aruco_left_base', 'joint_aruco_shoulder', 'joint_aruco_top_wrist', 'joint_aruco_inner_wrist', 'camera_joint', 'camera_link_joint', 'camera_depth_joint', 'camera_depth_optical_joint', 'camera_infra1_joint', 'camera_infra1_optical_joint', 'camera_infra2_joint', 'camera_infra2_optical_joint', 'camera_color_joint', 'camera_color_optical_joint', 'camera_accel_joint', 'camera_accel_optical_joint', 'camera_gyro_joint', 'camera_gyro_optical_joint', 'gripper_camera_joint', 'gripper_camera_link_joint', 'gripper_camera_depth_joint', 'gripper_camera_depth_optical_joint', 'gripper_camera_infra1_joint', 'gripper_camera_infra1_optical_joint', 'gripper_camera_infra2_joint', 'gripper_camera_infra2_optical_joint', 'gripper_camera_color_joint', 'gripper_camera_color_optical_joint', 'joint_laser', 'joint_base_imu', 'joint_respeaker', 'joint_wrist_quick_connect', 'joint_gripper_finger_right', 'joint_gripper_fingertip_right', 'joint_aruco_fingertip_right', 'joint_gripper_finger_left', 'joint_gripper_fingertip_left', 'joint_aruco_fingertip_left', 'joint_aruco_d405', 'joint_head_nav_cam'] 
    # joints_kept = ['joint_mast', 'joint_lift', 'joint_arm_l4', 'joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0', 'joint_wrist_yaw', 'joint_wrist_yaw_bottom', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_gripper_s3_body', 'joint_grasp_center']
    joints_to_remove = [l for l in modified_urdf._joints if l.name in names_of_joints_to_remove]
    for jr in joints_to_remove:
        modified_urdf._joints.remove(jr)


            
    joint_base_translation = urdfpy.Joint(name='joint_base_translation',
                                        parent='link_base_rotation',
                                        child='link_base_translation',
                                        joint_type='prismatic',
                                        axis=np.array([1.0, 0.0, 0.0]),
                                        origin=np.eye(4, dtype=np.float64),
                                        limit=urdfpy.JointLimit(effort=100.0, velocity=1.0, lower=-1.0, upper=1.0))
    modified_urdf._joints.append(joint_base_translation)
    link_base_translation = urdfpy.Link(name='link_base_translation',
                                        inertial=None,
                                        visuals=None,
                                        collisions=None)
    modified_urdf._links.append(link_base_translation)
    
        # Add virtual base joint
    joint_base_rotation = urdfpy.Joint(name='joint_base_rotation',
                                        parent='base_link',
                                        child='link_base_rotation',
                                        joint_type='revolute',
                                        axis=np.array([0.0, 0.0, 1.0]),
                                        origin=np.eye(4, dtype=np.float64),
                                        limit=urdfpy.JointLimit(effort=100.0, velocity=1.0, lower=-np.pi, upper=np.pi))
    modified_urdf._joints.append(joint_base_rotation)
    link_base_rotation = urdfpy.Link(name='link_base_rotation',
                                    inertial=None,
                                    visuals=None,
                                    collisions=None)
    modified_urdf._links.append(link_base_rotation)
    # amend the chain    
    for j in modified_urdf._joints:
        if j.name == 'joint_base_translation':
            j.parent = 'link_base_rotation'
            
            
    # amend the chain
    for j in modified_urdf._joints:
        if j.name == 'joint_mast':
            j.parent = 'link_base_translation'

    new_urdf_path = "/tmp/iktutorial/stretch.urdf"
    modified_urdf.save(new_urdf_path)

    chain = ikpy.chain.Chain.from_urdf_file(new_urdf_path)

    for link in chain.links:
        print(f"* Link Name: {link.name}, Type: {link.joint_type}")
        
    def get_lift_state(joint_states):
        joint_name = 'joint_lift'
        i = joint_states.name.index(joint_name)
        lift_position = joint_states.position[i]
        lift_velocity = joint_states.velocity[i]
        lift_effort = joint_states.effort[i]
        return [lift_position, lift_velocity, lift_effort]
    
    def get_arm_state(joint_states):
        joint_name = 'joint_arm_l0'
        i = joint_states.name.index(joint_name)
        arm_position = joint_states.position[i]
        arm_velocity = joint_states.velocity[i]
        arm_effort = joint_states.effort[i]
        return [arm_position, arm_velocity, arm_effort]
    
    def get_wrist_yaw_state(joint_states):
        joint_name = 'joint_wrist_yaw'
        i = joint_states.name.index(joint_name)
        wrist_yaw_position = joint_states.position[i]
        wrist_yaw_velocity = joint_states.velocity[i]
        wrist_yaw_effort = joint_states.effort[i]
        return [wrist_yaw_position, wrist_yaw_velocity, wrist_yaw_effort]
    
    def get_wrist_pitch_state(joint_states):
        joint_name = 'joint_wrist_pitch'
        i = joint_states.name.index(joint_name)
        wrist_pitch_position = joint_states.position[i]
        wrist_pitch_velocity = joint_states.velocity[i]
        wrist_pitch_effort = joint_states.effort[i]
        return [wrist_pitch_position, wrist_pitch_velocity, wrist_pitch_effort]
    
    def get_wrist_roll_state(joint_states):
        joint_name = 'joint_wrist_roll'
        i = joint_states.name.index(joint_name)
        wrist_roll_position = joint_states.position[i]
        wrist_roll_velocity = joint_states.velocity[i]
        wrist_roll_effort = joint_states.effort[i]
        return [wrist_roll_position, wrist_roll_velocity, wrist_roll_effort]

    def get_current_configuration():
        def bound_range(name, value):
            names = [l.name for l in chain.links]
            index = names.index(name)
            bounds = chain.links[index].bounds
            return min(max(value, bounds[0]), bounds[1])

        q_base = 0.0
        q_rotation = 0.0
        # print(get_lift_state(robot.joint_state))
        # print(get_arm_state(robot.joint_state))
        q_lift = bound_range('joint_lift', get_lift_state(robot.joint_state)[0])
        q_arml = bound_range('joint_arm_l0', get_arm_state(robot.joint_state)[0] / 4.0)
        q_yaw = bound_range('joint_wrist_yaw', get_wrist_yaw_state(robot.joint_state)[0])
        q_pitch = bound_range('joint_wrist_pitch', get_wrist_pitch_state(robot.joint_state)[0])
        q_roll = bound_range('joint_wrist_roll', get_wrist_roll_state(robot.joint_state)[0])
        return [0.0, q_rotation, q_base, 0.0 , q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]

    def move_to_configuration(q):
        q_rotation = q[1]
        q_base = q[2]
        q_lift = q[4]
        q_arm = q[9] + q[6] + q[7] + q[8]
        q_yaw = q[10]
        q_pitch = q[12]
        q_roll = q[13]
        # robot.base.translate_by(q_base)
        # robot.lift.move_to(q_lift)
        # robot.arm.move_to(q_arm)
        # robot.end_of_arm.move_to('wrist_yaw', q_yaw)
        # robot.end_of_arm.move_to('wrist_pitch', q_pitch)
        # robot.end_of_arm.move_to('wrist_roll', q_roll)
        # robot.push_command()
        print(f"Moving to configuration: base translation {q_base} base_rotation {q_rotation} lift {q_lift}, arm {q_arm}, wrist yaw {q_yaw}, wrist pitch {q_pitch}, wrist roll {q_roll}")
        robot.move_to_pose({'rotate_mobile_base': q_rotation}, blocking=True)
        robot.move_to_pose({'translate_mobile_base': q_base, 'joint_lift': q_lift, 'joint_arm': q_arm, 'joint_wrist_yaw': q_yaw, 'joint_wrist_pitch': q_pitch, 'joint_wrist_roll': q_roll}, blocking=True)
        
    def move_to_grasp_goal(target_point, target_orientation):
        q_init = get_current_configuration()
        q_soln = chain.inverse_kinematics(target_point, target_orientation, orientation_mode='all', initial_position=q_init)
        print('Solution:', q_soln)

        err = np.linalg.norm(chain.forward_kinematics(q_soln)[:3, 3] - target_point)
        if not np.isclose(err, 0.0, atol=1e-2):
            print("IKPy did not find a valid solution")
            return
        move_to_configuration(q=q_soln)
        return q_soln

    def get_current_grasp_pose():
        q = get_current_configuration()
        return chain.forward_kinematics(q)


    # robot.stow()
    global first_target_point
    move_to_grasp_goal(first_target_point, first_target_orientation)
    print(get_current_grasp_pose())
    
    move_to_grasp_goal(second_target_point, second_target_orientation)
    print(get_current_grasp_pose())
    
    move_to_grasp_goal(third_target_point, third_target_orientation)
    print(get_current_grasp_pose())
    
