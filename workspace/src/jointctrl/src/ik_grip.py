#!/usr/bin/env python3
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
# from intera_interface import gripper as robot_gripper

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# additional imports 
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as robotiq_output_msg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as robotiq_input_msg
import tf2_ros
import numpy as np


def lookup_gripper():
    tfBuffer = tf2_ros.Buffer() ## TODO: initialize a buffer
    tfListener = tf2_ros.TransformListener(tfBuffer) ## TODO: initialize a transform listener

    try:
      # TODO: lookup the transform and save it in trans
      # The rospy.Time(0) is the latest available
      # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
      trans = tfBuffer.lookup_transform('base', 'wrist_3_link', rospy.Time(0), rospy.Duration(10.0))
      print(trans)
    except Exception as e:
      print(e)
      print("Retrying ...")

    # grip_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    # return np.array(grip_pos)
    return trans.transform

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # rospy.init_node('gripper_test')

    # Set up the right gripper
    # right_gripper = robot_gripper.Gripper('right_gripper')

    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():

        # Open the right gripper
        # print('Opening...')
        # right_gripper.open()
        # rospy.sleep(1.0)
        # print('Done!')

        ### START STATE ###
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "manipulator"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "wrist_3_link"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        cur_pose = lookup_gripper()

        # Set the desired orientation for the end effector HERE
        # request.ik_request.pose_stamped.pose.position.x = 0.781
        # request.ik_request.pose_stamped.pose.position.y = 0.226
        # request.ik_request.pose_stamped.pose.position.z = -0.150      
        # request.ik_request.pose_stamped.pose.orientation.x = 0
        # request.ik_request.pose_stamped.pose.orientation.y = 1.0
        # request.ik_request.pose_stamped.pose.orientation.z = 0
        # request.ik_request.pose_stamped.pose.orientation.w = 0
        request.ik_request.pose_stamped.pose.position.x = cur_pose.translation.x
        request.ik_request.pose_stamped.pose.position.y = cur_pose.translation.y
        request.ik_request.pose_stamped.pose.position.z = cur_pose.translation.z
        request.ik_request.pose_stamped.pose.orientation.x = 0
        request.ik_request.pose_stamped.pose.orientation.y = 0
        request.ik_request.pose_stamped.pose.orientation.z = cur_pose.rotation.z
        request.ik_request.pose_stamped.pose.orientation.w = 0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("manipulator")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            # group.set_position_target([0.781, 0.226, -0.150])

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])

                # Close the right gripper
                # print('Closing...')
                # right_gripper.close()
                # rospy.sleep(1.0)
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)



        ### END STATE ###
        input('Press [ Enter ]: ')


# Python's syntax for a main() method
if __name__ == '__main__':
    main()
