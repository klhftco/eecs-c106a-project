#!/usr/bin/env python
# The line above tells Linux that this file is a Python script, and that the OS
# should use the Python interpreter in /usr/bin/env to run it. Don't forget to
# use "chmod +x [filename]" to make this script executable.

# Import the rospy package. For an import to work, it must be specified
# in both the package manifest AND the Python file in which it is used.
import rospy

# Import the String message type from the /msg directory of the std_msgs package.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Define the method which contains the node's main functionality
def talker():

    # Create an instance of the rospy.Publisher object which we can  use to
    # publish messages to a topic. This publisher publishes messages of type
    # std_msgs/String to the topic /chatter_talk
    pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
    
    # Create a timer object that will sleep long enough to result in a 10Hz
    # publishing rate
    r = rospy.Rate(5) # 10hz

    n = 0
    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        # Construct a string that we want to publish (in Python, the "%"
        # operator functions similarly to sprintf in C or MATLAB)
        command = JointTrajectory()
        command.header.stamp = rospy.Time.now()
        command.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        point_start = JointTrajectoryPoint()
        point_pick = JointTrajectoryPoint()
        point_hold = JointTrajectoryPoint()
        point_end = JointTrajectoryPoint()

        positions = [
            [-1.55, -1.6, -1.5, 4.7, 1.6, 2.85],
            [-1.52, -1.9, -1.5, 4.85, 1.6, 2.85],
            [-1.52, -1.9, -1.5, 4.85, 1.6, 2.85],
            [-1.55, -1.6, -1.5, 4.7, 1.6, 2.85],
            [-1.55, -1.6, -1.8, 4.7, 1.6, 2.85],
        ]

        point_array = []
        for i, pos in enumerate(positions):
            point = JointTrajectoryPoint()
            point.positions = pos
            point.time_from_start = rospy.Duration((i+1) * 5)
            point_array.append(point)

        command.points = point_array

        print(command)
        # Publish our string to the 'chatter_talk' topic
        pub.publish(command)
        if n == 3:
            break
        n += 1
        # print("Published to the topic!")
        
        # Use our rate object to sleep until it is time to publish again
        r.sleep()
            
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('talker', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    try:
        talker()
    except rospy.ROSInterruptException: pass