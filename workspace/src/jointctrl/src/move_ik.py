#!/usr/bin/env python3
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from geometry_msgs.msg import WrenchStamped as forceReading
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
import tf.transformations

class GripperCommander():
    def __init__(self):
        self.robotiq_gripper_pub = rospy.Publisher(
           "Robotiq2FGripperRobotOutput", robotiq_output_msg.Robotiq2FGripper_robot_output, queue_size=10)
        self.robotiq_gripper_sub = rospy.Subscriber(
            "Robotiq2FGripperRobotInput", robotiq_input_msg.Robotiq2FGripper_robot_input, self.gripper_status_callback)
        self.wrench_sub = rospy.Subscriber('/wrench_averaged', forceReading, self.wrench_callback)
        
        self.gripper_status = None
        self.wrench_status = None

    def gripper_status_callback(self, robotiq_input_msg):
        self.gripper_status = robotiq_input_msg

    def wrench_callback(self, wrench_msg):
        self.wrench_status = wrench_msg

    def send_gripper_command(self, rPR, rACT=1, rGTO=1, rATR=0, rSP=128, rFR=32):
        gripper_command = robotiq_output_msg.Robotiq2FGripper_robot_output()
        gripper_command.rACT = rACT # must remain at 1, will activate upon being switched to one
        gripper_command.rGTO = rGTO # 1 means it is following the go to routine
        gripper_command.rATR = rATR # set to 1 for automatic release routine
        gripper_command.rPR = min(rPR, 150)
        gripper_command.rSP = rSP # 1/2 max speed
        gripper_command.rFR = rFR # 1/4 max force
        self.robotiq_gripper_pub.publish(gripper_command)

    def activate_gripper(self):
        self.send_gripper_command(rPR=0, rACT=1, rGTO=0, rATR=0, rSP=0, rFR=0)
        rospy.sleep(0.5)
        self.send_gripper_command(rPR=0, rACT=0, rGTO=0, rATR=0, rSP=0, rFR=0)
        rospy.sleep(0.5)
        self.send_gripper_command(rPR=0, rACT=1, rGTO=0, rATR=0, rSP=128, rFR=48)
        rospy.sleep(0.5)

    def open_gripper(self):
        self.send_gripper_command(rPR=0, rSP=255, rFR=255)
        while self.gripper_status.gOBJ != 1 and self.gripper_status.gPO > 5:
            # print("gOBJ: " + str(self.gripper_status.gOBJ))
            # print("gPO: " + str(self.gripper_status.gPO))
            # print("")
            pass
        self.send_gripper_command(rPR=self.gripper_status.gPO, rGTO=0, rSP=16, rFR=255)

    def close_gripper(self):
        self.send_gripper_command(rPR=255, rSP=1, rFR=255)
        while self.gripper_status.gOBJ != 2 and self.gripper_status.gPO < 250:
            pass
        self.send_gripper_command(rPR=self.gripper_status.gPO, rGTO=0, rSP=1, rFR=255)

    def close_num(self, position):
        # if gripper.status.rACT == 0: give warining to activate
        self.send_gripper_command(rPR=position, rGTO=1, rSP=255, rFR=255)

    def lookup_gripper(self):
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer) 
        try:
            trans = tfBuffer.lookup_transform('base_link', 'tool0', rospy.Time(0), rospy.Duration(10.0))
        except Exception as e:
            print(e)
            print("Retrying ...")
        return trans.transforms

class Plan():
    # tuck -> grab -> down -> up -> dest -> down -> release -> up -> tuck
    def __init__(self):
        self.rotate = 1
        self.WEIGHT_THRESHOLD = 10.0
        self.SPRING_THRESHOLD = 10.0
        self.LOWER_Z_LIMIT = 0.32
        # NOTE: poses are in meters

        straight_down = Quaternion()
        straight_down.x = 1.0
        straight_down.y = 0.0
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

        self.probe = Pose()
        self.probe.position.x = 0.25
        self.probe.position.y = 0.69
        self.probe.position.z = 0.37  # Cube 0.37, box 0.41, sponge 0.34
        self.probe.orientation = straight_down

        self.probe_down = Pose()
        self.probe_down.position.x = 0.25
        self.probe_down.position.y = 0.69
        self.probe_down.position.z = 0.36  # Cube 0.36, box 0.40, sponge 0.33
        self.probe_down.orientation = straight_down

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

    def get_move_plan(self, weight=10.0, spring=10.0):
        '''
        Input:
        - Weight: weight
        - Spring Constant: spring

        Returns output as boolean shouldPush
        '''
        # Consider 4 cases:
        # If heavy, rigid -> push
        # If heavy, deformable -> pick
        # If light, rigid -> push/pick
        # If light, deformable -> pick

        obj = self.lookup_tag(7)
        dest = self.lookup_tag(6)
        return self.return_pick_plan(obj, dest)

        # if heavy, push
        if weight > self.WEIGHT_THRESHOLD:
            if spring > self.SPRING_THRESHOLD:
                return True
            else:
                return False
        else: # weight <= threshold
            if spring > self.SPRING_THRESHOLD:
                return True
            else:
                return False

    def if_fail(self):
        newPlan = None
        return newPlan

    def lookup_tag(self, tag_number):
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        try:
            trans = tfBuffer.lookup_transform('base_link', f'ar_marker_{tag_number}', rospy.Time(0), rospy.Duration(10.0))
        except Exception as e:
            print(e)
            print("Retrying ...")

        quat = [getattr(trans.transform.rotation, dim) for dim in ('x', 'y', 'z', 'w')]
        euler = [-np.pi, 0, self.rotate * tf.transformations.euler_from_quaternion(quat)[2] - np.pi]
        quat = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
        self.rotate *= -1

        rot = Quaternion()
        rot.x = quat[0]
        rot.y = quat[1]
        rot.z = quat[2]
        rot.w = quat[3]
        trans.transform.rotation = rot

        return trans.transform

    def get_lifted_pose(self, pose):
        ret = Pose()
        ret.position.x = pose.position.x
        ret.position.y = pose.position.y
        ret.position.z = pose.position.z + 0.1
        ret.orientation = pose.orientation
        return ret

    def get_probe_pose(self, pose):
        ret1 = Pose()
        ret1.position.x = pose.position.x
        ret1.position.y = pose.position.y
        ret1.position.z = pose.position.z + 0.005
        ret1.orientation = pose.orientation

        ret2 = Pose()
        ret2.position.x = pose.position.x
        ret2.position.y = pose.position.y
        ret2.position.z = pose.position.z - 0.005
        ret2.orientation = pose.orientation

        return ret1, ret2

    def return_pick_plan(self, obj_trans, dest_trans):
        obj = Pose()
        obj.position.x = obj_trans.translation.x
        obj.position.y = obj_trans.translation.y
        obj.position.z = max(obj_trans.translation.z, self.LOWER_Z_LIMIT)
        obj.orientation = obj_trans.rotation

        dest = Pose()
        dest.position.x = dest_trans.translation.x
        dest.position.y = dest_trans.translation.y
        dest.position.z = max(dest_trans.translation.z, self.LOWER_Z_LIMIT)
        dest.orientation = dest_trans.rotation

        lifted_obj = self.get_lifted_pose(obj)
        lifted_dest = self.get_lifted_pose(dest)

        # print(dest)
        # print(lifted_dest)
        # print(obj)
        # print(lifted_obj)

        return [lifted_obj,
                obj,
                1,
                lifted_obj,
                lifted_dest,
                dest,
                0,
                lifted_dest,
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

    def return_probe_plan(self):
        return [self.probe,
                255,
                self.probe_down,
                4]  # 4 means to write the deformability constant in `deformability`
    
    def return_weigh_plan(self):
        return [self.pick,
                0,
                2,  # 2 means to save the current reading of force.z
                self.grab,
                1,
                self.pick,
                3,  # 3 means to calculate the delta in force.z with and without object
                5,  # 5 means to delete our first data point
                self.grab,
                0,
                self.pick,
                2,
                self.grab,
                1,
                self.pick,
                3,
                self.grab,
                0,
                self.pick,
                2,
                self.grab,
                1,
                self.pick,
                3,
                self.grab,
                0,
                self.pick,
                2,
                self.grab,
                1,
                self.pick,
                3,
                self.grab,
                0,
                self.pick,
                2,
                self.grab,
                1,
                self.pick,
                3,
                self.grab,
                0,
                self.pick,
                2,
                self.grab,
                1,
                self.pick,
                3,
                6,  # 6 means to calculate the average delta and write it in `weight`
                self.grab]

    def return_test_plan(self):
        return [self.tuck,
                self.pick,
                2,
                self.grab,
                1,
                self.pick,
                3,
                self.grab,
                0,
                self.pick,
                2,
                self.grab,
                1,
                self.pick,
                3,
                self.grab,
                0,
                self.pick,
                2,
                self.grab,
                1,
                self.pick,
                3,
                self.grab,
                0,
                self.pick,
                2,
                self.grab,
                1,
                self.pick,
                3,
                self.grab,
                0,
                self.pick,
                2,
                self.grab,
                1,
                self.pick,
                3,
                self.grab,
                0,
                self.pick,
                self.tuck]

class ur5_actuator():
    def __init__(self):
        self.pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
        self.rate = None
        self.gripper = GripperCommander()
        self.move_group = MoveGroupCommander("manipulator")
        self.planner = Plan()
        self.compute_ik = None

    def get_ik(self, pose):
        request = GetPositionIKRequest()
        request.ik_request.group_name = "manipulator"
        request.ik_request.ik_link_name = "tool0"
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.pose = pose
        try:
            response = self.compute_ik(request)
            self.move_group.set_pose_target(request.ik_request.pose_stamped)

            trajectory = self.move_group.plan()[1].joint_trajectory
            # print(trajectory)
            user_input = ''
            while (user_input != 'z'):
                user_input = input("Enter 'z' if the trajectory looks safe on RVIZ")

            # Execute IK if safe
            if user_input == 'z':
                self.pub.publish(trajectory)
                print("Published to the topic!")
                

            # Use our rate object to sleep until it is time to publish again
            self.rate.sleep()

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def run(self):
        # Wait for the IK service to become available
        print("Started the program!")
        rospy.wait_for_service('compute_ik')
        print("compute_ik service is running.")
        rospy.init_node('service_query')
        self.rate = rospy.Rate(5)
        # Create the function used to call the service
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)    

        self.gripper.activate_gripper()
        self.gripper.open_gripper()
        self.get_ik(self.planner.tuck)

        curr_force = 0  # Variable to hold the force reading when arm is in "pick" position
        weight_samples = 0
        weight = 0
        deformability = 0

        # Probe routine
        for move in self.planner.return_probe_plan():
            print(move)
            if rospy.is_shutdown():
                break
            elif move == 4:  # Get probe measurement
                rospy.sleep(2)
                deformability = self.gripper.wrench_status.wrench.force.z
                print("Deformability constant:", deformability)
            elif (move == 255) or (move == 120):
                rospy.sleep(5)
                self.gripper.close_num(move)
            else:
                input("Press enter for next move")
                self.get_ik(move)
        
        # If deformability < -85, it's solid
        # If deformability >= -85, it's deformable
        
        # Weigh Routine
        for move in self.planner.return_weigh_plan():
            print(move)
            if rospy.is_shutdown():
                break

            elif move == 0:
                rospy.sleep(3)
                self.gripper.open_gripper()
            elif move == 1:
                rospy.sleep(3)
                self.gripper.close_gripper()
            elif move == 2:
                rospy.sleep(1)
                curr_force = self.gripper.wrench_status.wrench.force.z
            elif move == 3:
                rospy.sleep(0.5)
                weight_samples += self.gripper.wrench_status.wrench.force.z - curr_force
            elif move == 5:
                weight_samples = 0
            elif move == 6:
                weight = weight_samples / 5
                print("Average delta z-force:", weight)
            else:
                input("Press enter for next move")
                self.get_ik(move)

        # If weight < 9, it's light
        # If weight >= 9, it's heavy



        for move in self.planner.get_move_plan():
            print(move)
            if rospy.is_shutdown():
                break

            if move == 0:
                self.gripper.open_gripper()
            elif move == 1:
                rospy.sleep(0.5)
                self.gripper.close_gripper()
            elif move == 2:
                rospy.sleep(1)
                curr_force = self.gripper.wrench_status.wrench.force.z
            elif move == 3:
                rospy.sleep(0.5)
                weight = self.gripper.wrench_status.wrench.force.z - curr_force
                print("Delta z-force:", weight)
            elif (move == 255) or (move == 120):
                self.gripper.close_num(move)
            else:
                input("Press enter for next move")
                self.get_ik(move)
    
def main():
    ur5 = ur5_actuator()
    
    input("Press enter to begin")
    ur5.run()

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
