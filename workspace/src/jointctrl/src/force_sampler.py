#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Wrench, WrenchStamped # Replace 'your_package' and 'ForceMessage' with your actual package and message type
import numpy as np

class ForceSubscriber:
    def __init__(self):
        self.force_sum = np.array([0.0, 0.0, 0.0])  # Initialize the sum of forces
        self.message_count = 0
        self.average_count = 25
        self.average_force = np.array([0.0, 0.0, 0.0])

        # Initialize ROS node and subscriber
        rospy.init_node('force_subscriber', anonymous=True)
        rospy.Subscriber('/wrench', WrenchStamped, self.force_callback)  # Replace 'force_topic' with your actual topic

        # Initialize ROS publisher for the average force
        self.average_force_publisher = rospy.Publisher('/wrench_averaged', Wrench, queue_size=10)  # Replace 'average_force_topic' with your desired topic

    def force_callback(self, data):
        # Callback function to process incoming force messages
        self.force_sum[0] += data.wrench.force.x
        self.force_sum[1] += data.wrench.force.y
        self.force_sum[2] += data.wrench.force.z
        self.message_count += 1

        if self.message_count == self.average_count:
            # Calculate average force
            self.average_force = self.force_sum / self.average_count

            # Reset variables for the next set of messages
            self.force_sum = [0.0, 0.0, 0.0]
            self.message_count = 0

            # Create and publish average force
            average_force_msg = Wrench()
            average_force_msg.force.x = self.average_force[0]
            average_force_msg.force.y = self.average_force[1]
            average_force_msg.force.z = self.average_force[2]
            self.average_force_publisher.publish(average_force_msg)

            # Do something with the average force, e.g., print it
            print("Average Force: {}".format(self.average_force))

if __name__ == '__main__':
    force_subscriber = ForceSubscriber()

    # Keep the program alive
    rospy.spin()
