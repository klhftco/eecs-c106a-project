#!/usr/bin/env python3
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

def main():
    # Wait for the IK service to become available
    print("Started the program!")
    rospy.wait_for_service('compute_ik')
    print("compute_ik service is running.")
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    # Create our own publisher to publist to the command topic
    pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
    r = rospy.Rate(5) # suggested lab code was 10Hz

    while not rospy.is_shutdown():
        # input('Press [ Enter ]: ')0
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "manipulator"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "tool0"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.5
        request.ik_request.pose_stamped.pose.position.y = 0.5
        request.ik_request.pose_stamped.pose.position.z = 0.4  # Keep the z-value greater than 0.35 at all times        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            group = MoveGroupCommander("manipulator")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ### group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            """
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan)
            """

            # Publish the trajectory manually
            for i in range(5):
                pub.publish(plan.joint_trajectory)
                print("Published to the topic!")
            
            # Use our rate object to sleep until it is time to publish again
            r.sleep()
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
