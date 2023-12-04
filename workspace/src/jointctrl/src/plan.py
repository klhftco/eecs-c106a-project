import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3

def wrench_to_weight(sensor_msg: WrenchStamped, calib_msg: WrenchStamped):
    force_0 = calib_msg.wrench.force
    force_1 = sensor_msg.wrench.force
    torque_0 = calib_msg.wrench.torque
    torque_1 = sensor_msg.wrench.torque

    force_0 = np.array([force_0.x, force_0.y, force_0.z])
    force_1 = np.array([force_1.x, force_1.y, force_1.z])
    weight = np.linalg.norm(force_1 - force_0)

    return weight

# wrench functions differ by direction of pushing
def wrench_to_spring(sensor_msg: WrenchStamped, calib_msg: WrenchStamped, sensor_pose, calib_pose):
    force_0 = calib_msg.wrench.force
    force_1 = sensor_msg.wrench.force
    torque_0 = calib_msg.wrench.torque
    torque_1 = sensor_msg.wrench.torque

    # Spring constant equation: F = kx (x = deflection)
    pos_0 = np.array([calib_pose.x, calib_pose.y, calib_pose.z])
    pos_1 = np.array([sensor_pose.x, sensor_pose.y, sensor_pose.z])
    deflection = np.linalg.norm(pos_1 - pos_0)
    
    force_0 = np.array([force_0.x, force_0.y, force_0.z])
    force_1 = np.array([force_1.x, force_1.y, force_1.z])
    rxn_force = np.linalg.norm(force_1 - force_0)
    
    # spring constant determination is not complete yet
    return rxn_force / deflection

def planner(weight, spring):
    '''
    Input: 
    - Weight: weight
    - Spring Constant: spring

    Returns output as boolean shouldPush
    '''
    WEIGHT_THRESHOLD = 10.0
    SPRING_THRESHOLD = 10.0

    # Consider 4 cases:
    # If heavy, rigid -> push
    # If heavy, deformable -> pick
    # If light, rigid -> push/pick
    # If light, deformable -> pick

    # if heavy, push
    if weight > WEIGHT_THRESHOLD:
        if spring > SPRING_THRESHOLD:
            return True
        else: 
            return False
    else: # weight <= threshold
        if spring > SPRING_THRESHOLD:
            return True
        else:
            return False

def main():
    # should be a service node

    # once sensor readings have been acquired, request service for planning
    # return plan score (True/False)
    # moveit side, convert plan to trajectory
    return