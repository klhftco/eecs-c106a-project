#!/usr/bin/env python3
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

import copy
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# additional imports
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as robotiq_output_msg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as robotiq_input_msg
import tf2_ros

# TODO: may need to adjust activate/open/close for tactual object detection
class GripperCommander():
    def __init__(self):
        self.robotiq_gripper_pub = rospy.Publisher(
           "Robotiq2FGripperRobotOutput", robotiq_output_msg.Robotiq2FGripper_robot_output, queue_size=10)
        self.robotiq_gripper_sub = rospy.Subscriber(
            "Robotiq2FGripperRobotInput", robotiq_input_msg.Robotiq2FGripper_robot_input, self.gripper_status_callback)

    def send_gripper_command(self, rPR, rACT=1, rGTO=1, rATR=0, rSP=128, rFR=64):
        gripper_command = robotiq_output_msg.Robotiq2FGripper_robot_output()
        gripper_command.rACT = rACT # must remain at 1, will activate upon being switched to one
        gripper_command.rGTO = rGTO # 1 means it is following the go to routine
        gripper_command.rATR = rATR # set to 1 for automatic release routine
        gripper_command.rPR = rPRFR = 150

        gripper_command.rSP = rSP # 1/2 max speed
        gripper_command.rFR = rFR / 2 # 1/4 max force
        self.robotiq_gripper_pub.publish(gripper_command)

    def gripper_status_callback(self, robotiq_input_msg):
        self.gripper_status = robotiq_input_msg

    def activate_gripper(self):
        # if gripper.status.rACT == 1: give warining to activate
        self.send_gripper_command(rPR=0, rACT=1, rGTO=1)

    def open_gripper(self):
        # if gripper.status.rACT == 0: give warining to activate
        self.send_gripper_command(rPR=0)

    def close_gripper(self):
        # if gripper.status.rACT == 0: give warining to activate
        self.send_gripper_command(rPR=255)

    def lookup_gripper(self):
        tfBuffer = tf2_ros.Buffer() ## TODO: initialize a buffer
        tfListener = tf2_ros.TransformListener(tfBuffer) ## TODO: initialize a transform listener

        try:
            # TODO: lookup the transform and save it in trans
            # The rospy.Time(0) is the latest available
            # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
            trans = tfBuffer.lookup_transform('base_link', 'tool0', rospy.Time(0), rospy.Duration(10.0))
            print(trans)
        except Exception as e:
            print(e)
            print("Retrying ...")

        # grip_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
        # return np.array(grip_pos)
        return trans.transform

class Plan():
    # tuck -> grab -> down -> up -> dest -> down -> release -> up -> tuck
    def __init__(self):
        straight_down = Quaternion()
        straight_down.x = 0.0
        straight_down.y = 1.0
        straight_down.z = 0.0
        straight_down.w = 0.0

        self.tuck = Pose()
        self.tuck.position.x = 0.0
        self.tuck.position.y = 0.3
        self.tuck.position.z = 0.5
        self.tuck.orientation = straight_down

        self.grab = Pose()
        self.grab.position.x = 0.25
        self.grab.position.y = 0.69
        self.grab.position.z = 0.32
        self.grab.orientation = straight_down

        self.grab_p = Pose()
        self.grab_p.position.x = 0.25
        self.grab_p.position.y = 0.69
        self.grab_p.position.z = 0.32 + 0.001
        self.grab_p.orientation = straight_down

        self.pick = Pose()
        self.pick.position.x = 0.25
        self.pick.position.y = 0.69
        self.pick.position.z = 0.32 + 0.1
        self.pick.orientation = straight_down

        self.drop = Pose()
        self.drop.position.x = -0.21
        self.drop.position.y = 0.69
        self.drop.position.z = 0.32 + 0.1
        self.drop.orientation = straight_down

        self.dest = Pose()
        self.dest.position.x = -0.21
        self.dest.position.y = 0.69
        self.dest.position.z = 0.32
        self.dest.orientation = straight_down

        self.dest_p = Pose()
        self.dest_p.position.x = -0.21
        self.dest_p.position.y = 0.69
        self.dest_p.position.z = 0.32 + 0.001
        self.dest_p.orientation = straight_down

    def planner(self, weight, spring):
        shouldPush = False
        return shouldPush

    def if_fail(self):
        newPlan = None
        return newPlan

    def return_pick_plan(self):
        return [self.tuck,
                self.pick,
                self.grab,
                1,
                self.pick,
                self.drop,
                self.dest,
                0,
                self.drop,
                self.tuck]

    def return_push_plan(self):
        return [self.tuck,
                self.pick,
                self.grab,
                1,
                self.pick,
                self.grab_p,
                self.dest_p,
                0,
                self.drop,
                self.tuck]

    def return_test_plan(self);
        return [self.tuck,
                self.pick,
                self.grab,
                1,
                self.pick,
                self.grab,
                0,
                self.pick,
                self.tuck]

def main():
    # Wait for the IK service to become available
    print("Started the program!")
    rospy.wait_for_service('compute_ik')
    print("compute_ik service is running.")
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    gripper = GripperCommander()
    gripper.activate_gripper()
    gripper.open_gripper()

    # Create our own publisher to publist to the command topic
    pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
    r = rospy.Rate(5) # suggested lab code was 10Hz FR = 150

    complete = False
    plan = Plan().return_test_plan()
    move_index = 0

    input("Press enter to begin")
    for move in plan:
        if rospy.is_shutdown():
            break;

        input("Press enter for next move")
        print(move)

        # if move is a number, operate gripper appropriately
        if move == 0:
            gripper.open_gripper()
        elif move == 1:
            gripper.close_gripper()
        else: # otherwise, construct IK request and compute path between points
            # Construct the request
            request = GetPositionIKRequest()
            request.ik_request.group_name = "manipulator"
            request.ik_request.ik_link_name = "tool0"
            request.ik_request.pose_stamped.header.frame_id = "base_link"
            request.ik_request.pose_stamped.pose = move

            try:
                response = compute_ik(request)
                group = MoveGroupCommander("manipulator")
                group.set_pose_target(request.ik_request.pose_stamped)

                # Plan IK
                plan = group.plan()[1].joint_trajectory
                user_input = input("Enter 'z' if the trajectory looks safe on RVIZ")

                # Execute IK if safe
                if user_input == 'z':
                    pub.publish(plan)
                    print("Published to the topic!")

                # Use our rate object to sleep until it is time to publish again
                r.sleep()

            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()


# def temp():
#     while not rospy.is_shutdown() and not complete:

#         input('Press [ Enter ] to begin planning: ')

#         # Construct the request
#         request = GetPositionIKRequest()
#         request.ik_request.group_name = "manipulator"
#         request.ik_request.ik_link_name = "tool0"
#         # request.ik_request.attempts = 20
#         request.ik_request.pose_stamped.header.frame_id = "base_link"

#         input('Press [ Enter ] to check location of gripper')
#         cur_pose = gripper.lookup_gripper()

#         input('Press [ Enter ] to plan trajectory of gripper to desired location')

#         # Set the desired orientation for the end effector HERE
#         request.ik_request.pose_stamped.pose.position.x = 0.0
#         request.ik_request.pose_stamped.pose.position.y = 0.3
#         request.ik_request.pose_stamped.pose.position.z = 0.5  # Keep the z-value greater than 0.35 at all times
#         request.ik_request.pose_stamped.pose.orientation.x = 0.0
#         request.ik_request.pose_stamped.pose.orientation.y = 1.0
#         request.ik_request.pose_stamped.pose.orientation.z = 0.0
#         request.ik_request.pose_stamped.pose.orientation.w = 0.0

#         try:
#             # Send the request to the service
#             response = compute_ik(request)

#             # Print the response HERE
#             group = MoveGroupCommander("manipulator")

#             # Setting position and orientation target
#             group.set_pose_target(request.ik_request.pose_stamped)

#             # TRY THIS
#             # Setting just the position without specifying the orientation
#             ### group.set_position_target([0.5, 0.5, 0.0])

#             # Plan IK
#             plan = group.plan()[1].joint_trajectory
#             # print(type(plan[0]))
#             # print(type(plan[1]))
#             # print(type(plan[2]))
#             print(plan)
#             user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")

#             # Execute IK if safe
#             if user_input == 'y':
#                 # group.execute(plan)
#                 pub.publish(plan)
#                 print("Published to the topic!")

#             # Use our rate object to sleep until it is time to publish again
#             r.sleep()

#         except rospy.ServiceException as e:
#             print("Service call failed: %s"%e)
