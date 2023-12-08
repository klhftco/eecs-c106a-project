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
from control_msgs.msg import JointTrajectoryControllerState

class GripperCommander():
    def __init__(self):
        self.robotiq_gripper_pub = rospy.Publisher(
           "Robotiq2FGripperRobotOutput", robotiq_output_msg.Robotiq2FGripper_robot_output, queue_size=10)
        self.robotiq_gripper_sub = rospy.Subscriber(
            "Robotiq2FGripperRobotInput", robotiq_input_msg.Robotiq2FGripper_robot_input, self.gripper_status_callback)
        self.wrench_sub = rospy.Subscriber('/wrench_averaged', forceReading, self.wrench_callback)
        
        self.gripper_status = None
        self.wrench_status = None
        self.min_grip = 142

    def gripper_status_callback(self, robotiq_input_msg):
        self.gripper_status = robotiq_input_msg

    def wrench_callback(self, wrench_msg):
        self.wrench_status = wrench_msg

    def send_gripper_command(self, rPR, rACT=1, rGTO=1, rATR=0, rSP=128, rFR=32):
        gripper_command = robotiq_output_msg.Robotiq2FGripper_robot_output()
        gripper_command.rACT = rACT # must remain at 1, will activate upon being switched to one
        gripper_command.rGTO = rGTO # 1 means it is following the go to routine
        gripper_command.rATR = rATR # set to 1 for automatic release routine
        gripper_command.rPR = rPR
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

    def close_gripper(self):
        self.send_gripper_command(rPR=self.min_grip, rSP=1, rFR=255)
        while self.gripper_status.gOBJ != 2 and self.gripper_status.gPO <= 140:
            pass
        self.send_gripper_command(rPR=self.gripper_status.gPO, rGTO=0, rSP=128, rFR=255)

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
        self.WEIGHT_THRESHOLD = 7.0
        self.SPRING_THRESHOLD = -35.0
        self.FRICTION_THRESHOLD = 10
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

        self.pick = None
        self.grab = None
        self.dest = None
        self.drop = None
        self.probe = None
        self.probe_down = None

    def locate_objects(self, obj_ar, dest_ar):
        obj = self.lookup_tag(obj_ar)
        obj_pose = Pose()
        obj_pose.position.x = obj.translation.x
        obj_pose.position.y = obj.translation.y - 0.00
        obj_pose.position.z = obj.translation.z - 0.06 + .07
        obj_pose.orientation = obj.rotation
        self.grab, self.pick = self.get_pick_pose(obj_pose)
        self.probe, self.probe_down = self.get_probe_pose(obj_pose)
        print(self.probe)

        dest = self.lookup_tag(dest_ar)
        dest_pose = Pose()
        dest_pose.position.x = dest.translation.x
        dest_pose.position.y = dest.translation.y
        dest_pose.position.z = dest.translation.z
        dest_pose.orientation = dest.rotation
        self.dest, self.drop = self.get_pick_pose(dest_pose)
        self.dest.position.z = self.grab.position.z

    def get_move_plan(self, weight=10.0, spring=10.0, friction=0.0):
        '''
        Input:
        - Weight: weight
        - Spring Constant: spring

        Returns plan corresponding to planning score
        '''
        # Consider 4 cases:
        # If heavy, rigid -> push
        # If heavy, deformable -> push
        # If light, rigid -> pick
        # If light, deformable -> push
        # If sticky -> pick

        # Empirical data:
        # If deformability < -85, it's solid
        # If deformability >= -85, it's deformable
        # If weight < 9, it's light
        # If weight >= 9, it's heavy

        c_weight = 1 / self.WEIGHT_THRESHOLD # > 1 if heavy
        c_spring = -1 / self.SPRING_THRESHOLD # > 1 if deformable
        c_friction = -1 / self.FRICTION_THRESHOLD

        planning_score = c_weight * np.abs(weight) + c_spring * max(0, spring) + c_friction * friction
        if (planning_score / 2) > 1:
            print(planning_score / 2, ": Push")
            return self.return_push_plan()
        else: 
            print(planning_score, ": Pick")
            return self.return_pick_plan()

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
        yaw = tf.transformations.euler_from_quaternion(quat)[2]
        yaw = np.deg2rad(np.rad2deg(yaw) % 90)
        euler = [-np.pi, 0, yaw]
        quat = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
        self.rotate *= -1

        rot = Quaternion()
        rot.x = quat[0]
        rot.y = quat[1]
        rot.z = quat[2]
        rot.w = quat[3]
        trans.transform.rotation = rot

        return trans.transform

    def get_pick_pose(self, pose):
        ret1 = Pose()
        ret1.position.x = pose.position.x
        ret1.position.y = pose.position.y
        ret1.position.z = max(pose.position.z + 0.24, self.LOWER_Z_LIMIT)
        ret1.orientation = pose.orientation

        ret2 = Pose()
        ret2.position.x = pose.position.x
        ret2.position.y = pose.position.y
        ret2.position.z = ret1.position.z + 0.1
        ret2.orientation = pose.orientation

        return ret1, ret2 # (grab, lift)

    def get_probe_pose(self, pose):
        ret1 = Pose()
        ret1.position.x = pose.position.x
        ret1.position.y = pose.position.y
        ret1.position.z = max(pose.position.z + 0.296, self.LOWER_Z_LIMIT + 0.01) + 0.005
        ret1.orientation = pose.orientation

        ret2 = Pose()
        ret2.position.x = pose.position.x
        ret2.position.y = pose.position.y
        ret2.position.z = ret1.position.z - 0.01
        ret2.orientation = pose.orientation

        return ret1, ret2 # (probe_start, probe_end)

    def update_pick_pose(self, pose):
        pose.position.z = pose.position.z - 0.296
        self.grab, self.pick = self.get_pick_pose(pose)
        self.dest.position.z = self.grab.position.z

    def return_pick_plan(self):
        return [# already in grab
                1,
                self.pick,
                self.drop,
                self.dest,
                0,
                self.drop,
                self.tuck]

    def return_push_plan(self):
        return [# already in grab
                1,
                self.dest,
                0,
                self.drop,
                self.tuck]
    
    def return_open_plan(self):
        return [self.pick,
                0,
                self.grab]

    def return_probe_plan(self):
        return [self.probe,
                255,
                4]  # 4 means to go down until deform < 0 and write the deformability constant in `deformability`
    
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
        self.is_steady = True
        self.steady_sub = rospy.Subscriber("/scaled_pos_joint_traj_controller/state", JointTrajectoryControllerState, self.is_steady_callback)

    def is_steady_callback(self, control_msg):
        self.is_steady = np.allclose(control_msg.actual.velocities, 
            np.zeros(len(control_msg.actual.velocities)), rtol=1e-3)

    def wait_for_steady(self):
        rospy.sleep(0.5)
        while(not self.is_steady):
                pass

    def get_ik(self, pose, safety=True):
        try:
            user_input = ''
            while (user_input != 'z'):
                request = GetPositionIKRequest()
                request.ik_request.group_name = "manipulator"
                request.ik_request.ik_link_name = "tool0"
                request.ik_request.pose_stamped.header.frame_id = "base_link"
                request.ik_request.pose_stamped.pose = pose

                response = self.compute_ik(request)
                self.move_group.set_pose_target(request.ik_request.pose_stamped)

                trajectory = self.move_group.plan()[1].joint_trajectory
                # print(trajectory)
                if (not safety):
                    user_input = 'z'
                else:
                    user_input = input("Enter 'z' if the trajectory looks safe on RVIZ")
                
            # Execute IK if safe
            if user_input == 'z' or not safety:
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

        self.get_ik(self.planner.tuck)
        self.gripper.activate_gripper()
        self.gripper.open_gripper()

        obj_ar = int(input("Object AR Marker #: "))
        dest_ar = 6
        self.planner.locate_objects(obj_ar, dest_ar)

        curr_force = 0  # Variable to hold the force reading when arm is in "pick" position
        weight_samples = 0
        weight = 0
        deformability = 0

        # Probe routine
        print("Begin probe")
        for i, move in enumerate(self.planner.return_probe_plan()):
            print(move)
            self.wait_for_steady()
            if rospy.is_shutdown():
                break
            elif self.gripper.wrench_status.wrench.force.z < 0:
                deformability = self.gripper.wrench_status.wrench.force.z
                self.get_ik(self.planner.probe)
                break;
            elif move == 4:  # Get probe measurement
                self.get_ik(self.planner.probe_down)
                self.wait_for_steady()
                deformability = self.gripper.wrench_status.wrench.force.z
                # while deformability > 0:
                #     self.planner.probe_down.position.z -= 0.005
                #     self.get_ik(self.planner.probe_down, safety=False)
                #     self.wait_for_steady()
                #     deformability = self.gripper.wrench_status.wrench.force.z    
                self.gripper.open_gripper()
                print("Deformability constant:", deformability)
                self.planner.update_pick_pose(self.planner.probe_down)
            elif (move == 255) or (move == 120):
                self.gripper.close_num(move)
            else:
                input("Press enter for next move")
                self.get_ik(move)

        if deformability < -35:
            # Weigh Routine
            print("Begin weigh")
            for i, move in enumerate(self.planner.return_weigh_plan()):
                self.wait_for_steady()
                safety = False
                if i < 4:
                    safety = True
                print(move)
                if rospy.is_shutdown():
                    break

                elif move == 0:
                    self.gripper.open_gripper()
                elif move == 1:
                    self.gripper.close_gripper()
                elif move == 2:
                    curr_force = self.gripper.wrench_status.wrench.force.z
                elif move == 3:
                    weight_samples += self.gripper.wrench_status.wrench.force.z - curr_force
                elif move == 5:
                    weight_samples = 0
                elif move == 6:
                    weight = weight_samples / 5
                    print("Average delta z-force:", weight)
                else:
                    if safety:
                        input("Press enter for next move")
                    self.get_ik(move, safety=safety)
        else:
            for i, move in enumerate(self.planner.return_open_plan()):
                self.wait_for_steady()
                
                print(move)
                if rospy.is_shutdown():
                    break
                elif move == 0:
                    self.gripper.open_gripper()
                else:
                    input("Press enter for next move")
                    self.get_ik(move)
            weight = 50

        print("Begin move")
        for move in self.planner.get_move_plan(weight, deformability):
            self.wait_for_steady()
            print(move)
            if rospy.is_shutdown():
                break

            if move == 0:
                self.gripper.open_gripper()
            elif move == 1:
                self.gripper.close_gripper()
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
