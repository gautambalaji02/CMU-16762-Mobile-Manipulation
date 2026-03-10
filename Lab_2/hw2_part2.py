import rclpy, time
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from hello_helpers.hello_misc import HelloNode
from lab2_moveit import moveit2_utils

# Make sure to run `ros2 launch stretch_core stretch_driver.launch.py`
# Make sure to run `stretch_robot_home.py` before this script to reset odometry.

class MoveMe(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        self.main('move_me', 'move_me', wait_for_first_pointcloud=False)
        self.stow_the_robot()
        time.sleep(2.0)

        # Record actual stow positions from the robot
        self.stow_lift = self.get_joint_pos('joint_lift')
        self.stow_arm = [
            self.get_joint_pos('joint_arm_l3'),
            self.get_joint_pos('joint_arm_l2'),
            self.get_joint_pos('joint_arm_l1'),
            self.get_joint_pos('joint_arm_l0'),
        ]
        self.stow_wrist = [
            self.get_joint_pos('joint_wrist_yaw'),
            self.get_joint_pos('joint_wrist_pitch'),
            self.get_joint_pos('joint_wrist_roll'),
        ]

        print(f'Stow: lift={self.stow_lift:.3f}, arm={[f"{v:.3f}" for v in self.stow_arm]}, '
              f'wrist={[f"{v:.3f}" for v in self.stow_wrist]}')

        self.planning_group = 'mobile_base_arm'
        self.moveit, self.moveit_plan, self.planning_params = moveit2_utils.setup_moveit(self.planning_group)

       
        # Base: forward 0.2, left 0.2, rotate pi/2
        # Arm: lift to 0.5
        self.plan_and_execute_base(0.2, 0.2, np.pi / 2, 'Pose 0->1: Drive')
        self.plan_and_execute_arm(lift=0.5, label='Pose 0->1: Lift arm to 0.5m')

        # Base: forward 0.2, rotate pi/2
        # Arm: extend to 0.4m
        self.plan_and_execute_base(0.2, 0.0, np.pi / 2, 'Pose 1->2: Drive')
        self.plan_and_execute_arm(lift=0.5, arm=[0.1, 0.1, 0.1, 0.1], label='Pose 1->2: Extend arm')

        # Base: forward 0.2, rotate pi/2
        # Arm: rotate wrist 45 deg each
        self.plan_and_execute_base(0.2, 0.0, np.pi / 2, 'Pose 2->3: Drive')
        cur_yaw = self.get_joint_pos('joint_wrist_yaw')
        cur_pitch = self.get_joint_pos('joint_wrist_pitch')
        cur_roll = self.get_joint_pos('joint_wrist_roll')
        self.plan_and_execute_arm(
            lift=0.5, arm=[0.1, 0.1, 0.1, 0.1],
            wrist=[cur_yaw + np.pi/4, cur_pitch + np.pi/4, cur_roll + np.pi/4],
            label='Pose 2->3: Rotate wrist 45 deg'
        )

        # Arm: stow first (before driving, to avoid collision)
        # Base: forward 0.2, rotate pi/2 (back to start)
        self.plan_and_execute_arm(
            lift=self.stow_lift, arm=self.stow_arm, wrist=self.stow_wrist,
            label='Pose 3->4: Stow arm'
        )
        self.plan_and_execute_base(0.2, 0.0, np.pi / 2, 'Pose 3->4: Drive back')

        print('\n=== Done! ===')

    def plan_and_execute_base(self, x, y, theta, label):
        """Plan and execute a base-only motion. Arm joints stay at current positions."""
        print(f'\n--- {label} (x={x}, y={y}, theta={theta:.2f}) ---')
        self.print_base_state()

        goal_state = RobotState(self.moveit.get_robot_model())
        goal_state.set_joint_group_positions(self.planning_group, [
            x, y, theta,
            self.get_joint_pos('joint_lift'),
            self.get_joint_pos('joint_arm_l3'),
            self.get_joint_pos('joint_arm_l2'),
            self.get_joint_pos('joint_arm_l1'),
            self.get_joint_pos('joint_arm_l0'),
            self.get_joint_pos('joint_wrist_yaw'),
            self.get_joint_pos('joint_wrist_pitch'),
            self.get_joint_pos('joint_wrist_roll'),
        ])

        self.moveit_plan.set_start_state_to_current_state()
        self.moveit_plan.set_goal_state(robot_state=goal_state)
        plan = self.moveit_plan.plan(parameters=self.planning_params)

        if plan.trajectory is None:
            print(f'ERROR: Planning failed for {label}!')
            return False
        self.execute_plan(plan)
        time.sleep(2.0)
        return True

    def plan_and_execute_arm(self, lift=None, arm=None, wrist=None, label=''):
        """Plan and execute an arm-only motion. Base stays at current position (0,0,0)."""
        print(f'\n--- {label} ---')
        self.print_base_state()

        cur_lift = self.get_joint_pos('joint_lift') if lift is None else lift
        cur_arm = [
            self.get_joint_pos('joint_arm_l3'),
            self.get_joint_pos('joint_arm_l2'),
            self.get_joint_pos('joint_arm_l1'),
            self.get_joint_pos('joint_arm_l0'),
        ] if arm is None else arm
        cur_wrist = [
            self.get_joint_pos('joint_wrist_yaw'),
            self.get_joint_pos('joint_wrist_pitch'),
            self.get_joint_pos('joint_wrist_roll'),
        ] if wrist is None else wrist

        goal_state = RobotState(self.moveit.get_robot_model())
        goal_state.set_joint_group_positions(self.planning_group, [
            0.0, 0.0, 0.0,  # No base motion
            cur_lift,
            cur_arm[0], cur_arm[1], cur_arm[2], cur_arm[3],
            cur_wrist[0], cur_wrist[1], cur_wrist[2],
        ])

        self.moveit_plan.set_start_state_to_current_state()
        self.moveit_plan.set_goal_state(robot_state=goal_state)
        plan = self.moveit_plan.plan(parameters=self.planning_params)

        if plan.trajectory is None:
            print(f'ERROR: Planning failed for {label}!')
            return False
        self.execute_plan(plan)
        time.sleep(2.0)
        return True

    def print_base_state(self):
        """Print current base joint values for debugging bounds issues."""
        try:
            trans = self.get_joint_pos('translate_mobile_base')
            rot = self.get_joint_pos('rotate_mobile_base')
            print(f'  Current base state: translate={trans:.4f}, rotate={rot:.4f}')
        except (ValueError, IndexError):
            print('  Could not read base state')

    def execute_plan(self, plan):
        # You dont have to edit this part
        processor = moveit2_utils.TrajectoryProcessor()
        segments = processor.process_trajectory(plan, self.joint_state)

        for i, goal_traj in enumerate(segments):
            self.get_logger().info(f"Executing segment {i+1}/{len(segments)} (Mode: {self._detect_mode(goal_traj)})")

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = goal_traj

            future = self.trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().error(f"Segment {i+1} rejected!")
                break

            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            res = res_future.result()

            if res.result.error_code != res.result.SUCCESSFUL:
                self.get_logger().error(f"Segment {i+1} failed with code: {res.result.error_code}")
                break

    def get_joint_pos(self, joint_name):
        return self.joint_state.position[self.joint_state.name.index(joint_name)]

    def _detect_mode(self, traj):
        if 'translate_mobile_base' in traj.joint_names: return 'TRANSLATE'
        if 'rotate_mobile_base' in traj.joint_names: return 'ROTATE'
        return 'ARM_ONLY'

def main():
    MoveMe()

if __name__ == '__main__':
    main()