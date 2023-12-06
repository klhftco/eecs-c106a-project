#!/usr/bin/env python
# import xxlimited
import rospy

# Messages
from control_msgs.msg import JointTrajectoryControllerState
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped as forceReading

# Global variable for the received current position of the robot
v = 1
g = None

# Publishers for the desired trajectory and gripper
trajectory_pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
gripper_pub = rospy.Publisher("Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=100)

# Message to publish the desired trajectory
command = JointTrajectory()
command.joint_names = ["elbow_joint",   "shoulder_lift_joint", "shoulder_pan_joint",
                       "wrist_1_joint", "wrist_2_joint",       "wrist_3_joint"]
# Note that the `rostopic echo scaled_pos_joint_traj_controller/state` lists joint positions in the following order:
                        # [ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                        # "wrist_1_joint", "wrist_2_joint",      "wrist_3_joint"]
point = JointTrajectoryPoint()
positions = [  # Array of desired positions
    [-1.56, -1.61, -1.54, 4.69, 1.62, 2.85],  # Starting position
    [-1.25, -1.95, -1.21, 4.77, 1.53, 3.26],  # Hovering above the pickup position
    [-1.42, -1.94, -1.20, 4.95, 1.51, 3.27],  # Pick up position
    [-1.49, -1.83, -1.40, 5.38, 1.50, 3.27],  # Raised center position
    [-1.11, -1.93, -1.55, 4.58, 1.53, 4.57],  # Hovering above the drop position
    [-1.34, -1.95, -1.55, 4.86, 1.53, 3.00],  # Drop location (object will be a centimeter above the table)
    [-1.56, -1.61, -1.54, 4.69, 1.62, 2.85]   # End position (same as starting position)
]
# Probing pose to push down at Rubik's cube is -1.45, -1.85, -1.46, 4.91, 1.62, 2.85
# Probing pose to push down at foam sponge is -1.55, -1.88, -1.44, 5.01, 1.61, 2.85
num_positions = len(positions)

# Array of desired gripper positions, corresponding with the desired robot positions above
gripper_positions = [0, 0, 0, 150, 150, 150, 0]  

i = 0  # Index to show which desired position we're calling

# Commands to control the gripper
gripper_command = outputMsg.Robotiq2FGripper_robot_output()

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
Sunscriber function that listens to the gripper's object detection
sensors and determines whether or not an object has been detected.
"""
def object_detection(received_message):
    global g
    g = received_message

"""
Subscriber function that listens to the `wrench` node and prints the
x, y, and z forces read by the force sensor. Causes a lot of clutter
in the terminal.
"""
# def force(received_message):
#     print("force_x = " + str(received_message.wrench.force.x))
#     print("force_y = " + str(received_message.wrench.force.y))
#     print("force_z = " + str(received_message.wrench.force.z))
#     print("")

"""
Callback function that runs every 0.5 seconds to check if the velocity
of the robot is 0. If the velocity is 0, it publishes the next
desired trajectory and actuates the gripper as necessary.
"""
def timer_callback(event):
    global command, gripper_command, i, points
    if v <= 0.1:
        if i == num_positions:  # Has reached last listed desired position
            raise Exception("Reached the final desired position. Need to shut down.")

        if i == 1 or i == 0:  # If we successfully arrived at the starting position
            gripper_command.rACT = 1
            gripper_pub.publish(gripper_command)  # Active the gripper
            gripper_command.rACT = 0
            gripper_pub.publish(gripper_command)  # Active the gripper
            gripper_command.rACT = 1 # Activate
            gripper_pub.publish(gripper_command)  # Active the gripper
            gripper_command.rGTO = 1 # Start
            gripper_command.rSP = 255 # Speed
            gripper_command.rFR = 255 # Force
            gripper_pub.publish(gripper_command)  # Active the gripper

        # if gripper_positions[i] == 0 and g.gPR != 0:  # If gripper should be open, command it to open
        if gripper_positions[i] == 0:  # If gripper should be open, command it to open
            gripper_command.rGTO = 1 # Start
            gripper_command.rSP = 255 # Speed
            gripper_command.rFR = 255 # Force
            gripper_command.rPR = 0
            gripper_pub.publish(gripper_command)
        else:
            if g.gPR == 0:
                gripper_command.rGTO = 1 # Start
                gripper_command.rSP = 255 # Speed
                gripper_command.rFR = 255 # Force
                gripper_command.rPR = 50
                gripper_pub.publish(gripper_command)
                gripper_command.rSP = 1 # Speed
                gripper_command.rPR = 255
                gripper_pub.publish(gripper_command)
                while g.gOBJ != 2:
                    print("gOBJ: " + str(g.gOBJ))
                    print("gPO: " + str(g.gPO))
                    print("")
                    if g.gPO >= gripper_positions[i]:
                        break
                gripper_command.rGTO = 0 # Stop
                gripper_command.rPR = g.gPO
                gripper_pub.publish(gripper_command)


                # This sleep value and the one below it need to
                # add up to equal the polling rate for the timer
                # in the `main()` function.
                # rospy.sleep(0.25)  

        # rospy.sleep(0.125)

        command.header.stamp = rospy.Time.now()
        point.time_from_start = rospy.Duration((i+1) * 0.5)  # Change the 1 to 3 or 5 to make the robot move slower
        point.positions = positions[i]
        command.points = [point]

        for j in range(3):  # Publish desired trajectory three times because the arm's refresh rate is finnicky
            trajectory_pub.publish(command)
            # print(command)

        i += 1

"""
Function that starts the robot's movement sequence. Starts by opening
the gripper and then creating the subscriber node, which listens to
the topic where the robot's current position and velocities are
published. A timer calls the publisher function `timer_callback`
every 0.5 seconds.
"""
def main():
    # Start the subscriber nodes for the robot's current position, object detection, and force sensor readings
    rospy.Subscriber('/scaled_pos_joint_traj_controller/state', JointTrajectoryControllerState, subscriber_callback)
    rospy.Subscriber('/Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, object_detection)
    # rospy.Subscriber('/wrench', forceReading, force)

    # Initialize the timer that will call the publisher callback
    timer = rospy.Timer(rospy.Duration(0.25), timer_callback)
    rospy.spin()
    timer.shutdown()
          
# Terminal  
if __name__ == '__main__':
    rospy.init_node('main', anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException: pass
