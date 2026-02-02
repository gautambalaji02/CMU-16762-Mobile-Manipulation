import numpy as np
'''
Might have to change the import of stretch to:
import stretch_body.robot as robot

Depending on version control
'''
import stretch_body.body.stretch_body.robot as robot

def main():
    # Initialize the robot
	bot = robot.Robot()
	if(bot.startup()):
		print('Robot connected to hardware \n')
	else:
		print('Robot could not connect! \n')
		exit(0)

	# Let us first stow the robot
	print('Stowing the robot... \n')
	bot.stow()
	print('Robot stowed! \n')

	# Getting the soft limits of the arm and lift
	_, arm_max = bot.arm.soft_motion_limits['hard']
	_, lift_max = bot.lift.soft_motion_limits['hard']

	# Extend the telescoping arm all the way out and raise the lift all the way up at the same time
	print('Setting robot arm and lift... \n')		
	bot.arm.move_to(arm_max - 0.1)
	bot.lift.move_to(lift_max - 0.1)

	# Push and wait
	print('Extending arm and raising lift... \n')
	bot.push_command()
	bot.wait_command()

	# move all three of the wrist motors, one at a time
	print('Setting wrist motors individually... \n')

	print('Setting wrist_yaw to 45 degrees...')
	bot.end_of_arm.move_to('wrist_yaw', np.deg2rad(45))
	bot.push_command()
	bot.wait_command()

	print('Setting wrist_pitch to -30 degrees...')
	bot.end_of_arm.move_to('wrist_pitch', np.deg2rad(-30))
	bot.push_command()
	bot.wait_command()

	print('Setting wrist_roll to 90 degrees...')
	bot.end_of_arm.move_to('wrist_roll', np.deg2rad(90))
	bot.push_command()
	bot.wait_command()

	# Opening and closing the gripper
	print('Opening gripper to 100%... \n')
	bot.end_of_arm.move_to('stretch_gripper', 100)
	bot.push_command()
	bot.wait_command()

	print('Closing gripper to 0%... \n')
	bot.end_of_arm.move_to('stretch_gripper', -100)
	bot.push_command()
	bot.wait_command()

	# Rotating both motors of the real sense camera
	bot.head.move_to('head_pan', np.deg2rad(45))
	bot.head.move_to('head_tilt', np.deg2rad(-30))
	print('Rotating head pan and tilt... \n')
	bot.push_command()
	bot.wait_command()

	# Bringing robot back to stowed position
	bot.stow()
	print('Bringing robot back to stowed position... \n')
	print('Robot stowed! \n')

	# Driving the robot forward by 0.5 meters
	print('Driving robot forward by 0.5 meters... \n')
	bot.base.translate_by(0.5)
	bot.push_command()
	bot.wait_command()

	# Turning the base around by 180 degrees
	print('Turning robot around by 180 degrees... \n')
	bot.base.rotate_by(np.pi)
	bot.push_command()
	bot.wait_command()

	# Again driving robot back to original position
	print('Driving robot back to original position... \n')
	bot.base.translate_by(0.5)
	bot.push_command()
	bot.wait_command()

	# Shutting down the robot
	bot.stop()
	print('Robot shutdown complete. \n')

if __name__ == '__main__':
	main()