#!/usr/bin/env python
import rospy

# Messages
from control_msgs.msg import JointTrajectoryControllerState
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Global variable for the received current position of the robot
v = 1

# Publishers for the desired trajectory and gripper
trajectory_pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
gripper_pub = rospy.Publisher("Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=10)


# Message to publish the desired trajectory
command = JointTrajectory()
command.joint_names = ["elbow_joint",   "shoulder_lift_joint", "shoulder_pan_joint",
                       "wrist_1_joint", "wrist_2_joint",       "wrist_3_joint"]
# Note that the `rostopic echo scaled_pos_joint_traj_controller/state' lists joint positions in the following order:
                        # [ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                        # "wrist_1_joint", "wrist_2_joint",      "wrist_3_joint"]
point = JointTrajectoryPoint()
positions = [  # Array of desired positions
    [-1.56, -1.61, -1.54, 4.69, 1.62, 2.85],  # Starting position
    [-1.25, -1.95, -1.21, 4.77, 1.53, 3.26],  # Hovering above the pickup position
    [-1.42, -1.94, -1.20, 4.95, 1.51, 3.27],  # Pick up position
    [-1.49, -1.83, -1.40, 5.38, 1.50, 3.27],  # Raised center position
    [-1.11, -1.93, -1.55, 4.58, 1.53, 4.57],  # Hovering above the drop
    [-1.44, -1.94, -1.50, 4.98, 1.47, 1.45],  # Drop location
    [-1.56, -1.61, -1.54, 4.69, 1.62, 2.85]   # End position
]
# Probing pose to push down at Rubik's cube is -1.45, -1.85, -1.46, 4.91, 1.62, 2.85
# Probing pose to push down at foam sponge is -1.55, -1.88, -1.44, 5.01, 1.61, 2.85
num_positions = len(positions)


# Array of desired gripper positions, corresponding with the desired robot positions above
gripper_positions = [0, 0, 0, 140, 140, 140, 0]  


i = 0  # Index to show which desired position we're calling


gripper_command = outputMsg.Robotiq2FGripper_robot_output()
gripper_command.rACT = 1 # Activate
gripper_command.rGTO = 1 # Start
gripper_command.rSP = 200 # Speed
gripper_command.rFR = 100 # Force


"""
Subscriber fuction that listens to the robot's current position and
sets global variable v to estimate the current velocity of the robot.
"""
def subscriber_callback(received_message):
    global v
    velocities = received_message.actual.velocities
    v = sum([abs(vel) for vel in velocities])
    # print("v:", v)

"""
Callback function that runs every 1 second to check if the velocity
of the robot is 0. If the velocity is 0, it publishes the next
desired trajectory.
"""
def timer_callback(event):
    global command, i, points
    if v == 0:
        if i == num_positions:
            raise Exception("Reached the final desired position. Shutting down.")

        gripper_command.rPR = gripper_positions[i]
        gripper_pub.publish(gripper_command)
        rospy.sleep(1)

        command.header.stamp = rospy.Time.now()
        point.time_from_start = rospy.Duration((i+1) * 3)
        point.positions = positions[i]
        command.points = [point]

        for j in range(3):
            trajectory_pub.publish(command)
            print(command)

        i += 1


"""
Function that starts the robot's movement sequence. Starts by opening
the gripper and then creating the subscriber node, which listens to
the topic where the robot's current position and velocities are
published. A timer calls the publisher function `timer_callback`
every 1 second.
"""
def main():
    # Open the gripper
    global gripper_command
    gripper_command.rPR = 0
    gripper_pub.publish(gripper_command)

    # Start the subscriber node for the robot's current position
    rospy.Subscriber('/scaled_pos_joint_traj_controller/state', JointTrajectoryControllerState, subscriber_callback)
    
    # Initialize the timer that will call the publisher callback
    timer = rospy.Timer(rospy.Duration(2), timer_callback)
    rospy.spin()
    timer.shutdown()


          
# Terminal  
if __name__ == '__main__':
    rospy.init_node('main', anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException: pass
