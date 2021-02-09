#!/usr/bin/env python3
import pybullet as p
# import setup
import numpy as np
from math import isclose
import time
# from numpy.linalg import inv


class SceneObject:
    """
    SceneObject class
    """

    def __init__(self, object_id):
        """
        Constructor
        :param gripper_id: SceneObject ID derived from loading object to the scene
        """
        self.object_id = object_id
        self.pos, self.orn = p.getBasePositionAndOrientation(self.object_id)


class Manipulator(SceneObject):
    """
    Inherited from SceneObject
    """

    def __init__(self, gripper_id, open_fingers_pose, start_grasp_pos, key_names=None):
        """
        Create a gripper object using gripper id
        :param gripper_id:
        """
        self.gripper_id = gripper_id
        super(Manipulator, self).__init__(self.gripper_id)
        self.joint_info = []
        self.num_joints = p.getNumJoints(self.gripper_id)
        self.joint_indices = np.arange(1, self.num_joints)
        self.open_fingers_pose = open_fingers_pose
        self.start_grasp_pose = start_grasp_pos
        self.create_joint_dict(key_names)

    def create_joint_dict(self, keys):
        """
        Create a dictionary for easy referencing of joints
        :param keys: If None, uses nomenclature from urdf file, else uses nomenclature passed
        Note: 0 index is reserved for Base of the manipulator. Expected key for this is always 'Base'
        :return:
        """
        self.joint_dict = {}
        self.joint_dict_with_base = {}
        self.get_joints_info()
        if keys is not None:
            for key, value in zip(keys, self.joint_info):
                if key == 'Base':
                    self.joint_dict_with_base.update({key: value[0]})
                    continue
                self.joint_dict.update({key: value[0]})
                self.joint_dict_with_base.update({key: value[0]})
                # print ("KEY AND VALUE: {}, {}". format(key, value[0]))
        else:
            for i in range(0, len(self.joint_info)):
                if i == 0:
                    self.joint_dict_with_base.update({self.joint_info[i][1]: self.joint_info[i][0]})
                    continue
                self.joint_dict.update({self.joint_info[i][1]: self.joint_info[i][0]})
                self.joint_dict_with_base.update({self.joint_info[i][1]: self.joint_info[i][0]})
        # print("DICTIONARY CREATED:{}".format(self.joint_dict))
        # print("DICTIONARY CREATED WITH BASE:{}".format(self.joint_dict_with_base))

    def get_joints_info(self):
        """
        Get joint info of every joint of the manipulator
        :return: list of joint info of every joint
        """
        for joint in range(self.num_joints):
            self.joint_info.append(p.getJointInfo(self.gripper_id, joint))
        return self.joint_info

    def get_joint_angles(self):
        """
        Get the current pose(angle of joints
        Stores in self.curr_joint_angle : current joint angles as a list
        """
        self.curr_joint_poses = []
        curr_joint_states = p.getJointStates(self.gripper_id, self.joint_dict.values())#self.joint_indices)
        for joint in range(0, len(curr_joint_states)):
            # print("VALUES: {}, {}".format(joint, curr_joint_states[joint][0]))
            self.curr_joint_poses.append(curr_joint_states[joint][0])

    def pose_reached(self, joint_poses, abs_tol=1e-05):
        """
        Check if the joints have reached a given pose
        :param joint_poses: Given pose to reach
        :return: done : True if reached False otherwise
        """
        self.get_joint_angles()

        if len(self.curr_joint_poses) != len(joint_poses):
            print("Joint poses do not match number of joints!")
            raise ValueError
        else:
            count = 0
            for curr_joint, given_joint in zip(self.curr_joint_poses, joint_poses):
                # print("JOINTS: CURR: {} GIVEN {}:".format(curr_joint, given_joint))
                if isclose(curr_joint, given_joint, abs_tol=abs_tol):
                    count += 1
            if count == 4:
                done = True
            else:
                done = False
            return done

    def move_fingers_to_pose(self, joint_poses):
        """
        Position manipulator with fingers open (widespread)
        :param joint_poses: joint angles to go to
        :return: Boolean  value
        (True  -> action complete)
        (False -> action incomplete/erroneous)
        """
        done = False
        while p.isConnected() and not done:
            p.setJointMotorControlArray(bodyIndex=self.gripper_id, jointIndices=self.joint_indices,
                                controlMode=p.POSITION_CONTROL, targetPositions=joint_poses)
            # targetVelocities=v, forces=f)
            # Query joint angles and check if they match or are close to where we want them to be
            done = self.pose_reached(joint_poses)
            # time.sleep(1)
        return done

    def manipulate_object(self, cube, data):
        # Get Cube Pose curr and next from file
        # Get Contact Points btw Cube and End Effectors
        # Build matrix btw c_p and cube

        pass
