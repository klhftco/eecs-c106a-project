#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Wrench, WrenchStamped # Replace 'your_package' and 'ForceMessage' with your actual package and message type
import numpy as np

class ForceSubscriber:
    def __init__(self):
        # Initialize ROS node and subscriber
        rospy.init_node('ar_to_base', anonymous=True)

        # Initialize ROS publisher for the transform
        self.average_force_publisher = rospy.Publisher('/ar_to_base', WrenchStamped, queue_size=10)  # Replace 'average_force_topic' with your desired topic

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
            self.force_sum = np.array([0.0, 0.0, 0.0])
            self.message_count = 0

            # Create and publish average force
            average_force_msg = WrenchStamped()
            avg_force = Wrench()
            avg_force.force.x = self.average_force[0]
            avg_force.force.y = self.average_force[1]
            avg_force.force.z = self.average_force[2]
            average_force_msg.header = data.header
            average_force_msg.wrench = avg_force
            self.average_force_publisher.publish(average_force_msg)

            # Do something with the average force, e.g., print it
            print("Average Force: {}".format(self.average_force))

if __name__ == '__main__':
    force_subscriber = ForceSubscriber()

    # Keep the program alive
    rospy.spin()