#!/usr/bin/env python3

import pybullet as p
# import setup
import numpy as np
from math import isclose, radians
import Markers
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
        self.get_curr_pose()
        self.next_pos = None
        self.next_orn = None

    def get_curr_pose(self):
        self.curr_pos, self.curr_orn = p.getBasePositionAndOrientation(self.object_id)
        return self.curr_pos, self.curr_orn

    def get_next_pose(self, data, scale=0.1):
        """
        Get the next pose values from the file in the proper pose format
        :param data: Data line from file as a list [x, y, rotx,  f_x,f_y,f_rot_mag]
        :return:
        """
        self.next_pos = (data[1]*scale, data[0]*scale, self.curr_pos[2])
        # orn_eul = [0, 0, radians(data[2])]
        # self.next_orn = p.getQuaternionFromEuler(orn_eul)
        orn = ((1, 0, 0), radians(data[2]))
        self.next_orn = p.getQuaternionFromAxisAngle(orn[0], orn[1])


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
        curr_joint_states = p.getJointStates(self.gripper_id, self.joint_dict.values())  # self.joint_indices)
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
            if count == len(joint_poses):
                done = True
            else:
                done = False
            return done

    def get_contact_points(self, cube_id):
        """
        Get the contact points between object passed in (cube) and gripper
        If no  contact, returns None
        :param cube_id:
        :return: [contact_points_left, contact_points_right] left and right finger contacts
        """
        contact_points_info_left = p.getContactPoints(cube_id, self.gripper_id,
                                                      linkIndexB=self.joint_dict['L_Dist'])
        try:
            contact_points_left = contact_points_info_left[0][5]
        except IndexError:
            contact_points_left = None

        contact_points_info_right = p.getContactPoints(cube_id, self.gripper_id,
                                                       linkIndexB=self.joint_dict['R_Dist'])
        try:
            contact_points_right = contact_points_info_right[0][5]
        except IndexError:
            contact_points_right = None
        return [contact_points_left, contact_points_right]

    def move_fingers_to_pose(self, joint_poses, cube_id=None, abs_tol=1e-05):
        """
        Position manipulator with fingers open (widespread)
        :param joint_poses: joint angles to go to
        :param cube_id: If we want contact points between cube and gripper
        :param abs_tol: Absolute tolerance between pose reached and given pose
        :return: done: Boolean  value
        (True  -> action complete)
        :return contact_points: list containing left and right contact points
        """
        done = False
        original_distal_left = Markers.Marker()
        left_link = p.getLinkState(self.gripper_id, self.joint_dict['L_Dist'])
        distal_left_pos = left_link[4]
        distal_left_orn = left_link[5]
        # original_distal_left.set_marker_pose((distal_left_pos, distal_left_orn))

        while p.isConnected() and not done:
            p.stepSimulation()
            time.sleep(1. / 240.)
            p.setJointMotorControlArray(bodyIndex=self.gripper_id, jointIndices=self.joint_indices,
                                        controlMode=p.POSITION_CONTROL, targetPositions=joint_poses)
            # targetVelocities=v, forces=f)
            # Query joint angles and check if they match or are close to where we want them to be
            done = self.pose_reached(joint_poses, abs_tol)
            # time.sleep(1)
        if cube_id is not None:
            contact_points = self.get_contact_points(cube_id)

        else:
            contact_points = [None, None]
        return done, contact_points

    def get_pose_in_world_origin(self, cube, data):
        """
        Gets the new contact points in world coordinates
        :param cube: instance of object in scene class(object to move)
        :param data: line in file of human data [x,y,rmag,f_x,f_y,f_rot_mag]
        :return: list T_origin_newcontactpoints: next contact points in world coordinates for left and right
        """
        curr_contact_points = self.get_contact_points(cube.object_id)
        # print("CURRENT CONTACT POINTS: {}".format(curr_contact_points))
        cube.get_curr_pose()
        # print("CURRENT CUBE POSE IS: {}, {}".format(cube.curr_pos, cube.curr_orn))
        self.get_curr_pose()
        # print("CURRENT GRIPPER POSE IS: {}, {}".format(self.curr_pos, self.curr_orn))
        # print("DATAAAAAA: {}".format(data))
        cube.get_next_pose(data)
        # print("NEXT CUBE POSE IS: {}, {}".format(cube.next_pos, cube.next_orn))

        steady_orn = p.getQuaternionFromEuler([0,0,0])
        T_origin_nextpose_cube = p.multiplyTransforms(cube.curr_pos, cube.curr_orn, cube.next_pos, steady_orn)
        # T_origin_nextpose_cube = (T_origin_nextpose_cube_tuple[0], cube.next_orn)
        # print("NEW POINTS:{} \n {}".format(T_origin_nextpose_cube, T_origin_nextpose_cube_tuple))
        T_cube_origin = p.invertTransform(cube.curr_pos, cube.curr_orn)
        T_origin_newcontactpoints = []
        T_origin_newlink = []
        j = self.joint_dict['L_Dist']
        for i in range(0, len(curr_contact_points)):
            T_cube_contact_points = p.multiplyTransforms(T_cube_origin[0], T_cube_origin[1],
                                                         curr_contact_points[i], self.curr_orn)
            print("CONTACT POINTS IN CUBE TRANSFORM: {}".format(T_cube_contact_points))
            T_newcube_newcontactpoints = p.multiplyTransforms(T_cube_contact_points[0], T_cube_contact_points[1],
                                                              cube.next_pos, cube.next_orn)
            print("NEW CONTACT POINTS IN CUBE TRANSFORM: {}".format(T_newcube_newcontactpoints))
            T_origin_newcontactpoints.append(p.multiplyTransforms(T_origin_nextpose_cube[0], T_origin_nextpose_cube[1],
                                                             T_newcube_newcontactpoints[0], T_newcube_newcontactpoints[1]))
        # print("NEW CONTACT POINTS IN ORIGIN: {}".format(T_origin_newcontactpoints))
            T_cp_origin = p.invertTransform(curr_contact_points[i], self.curr_orn)
            left_link = p.getLinkState(self.gripper_id, j)
            distal_left_pos = left_link[4]
            distal_left_orn = left_link[5]
            T_cp_link = p.multiplyTransforms(T_cp_origin[0], T_cp_origin[1], distal_left_pos, distal_left_orn)
            T_origin_newlink.append(p.multiplyTransforms(T_origin_newcontactpoints[i][0], T_origin_newcontactpoints[i][1],
                                                    T_cp_link[0], T_cp_link[1]))
            j += 2

        return T_origin_newcontactpoints, T_origin_newlink, T_origin_nextpose_cube

    def manipulate_object(self, cube, data):
        """
        TODO: Dictionary keys should (maybe?) be a parameter?
        Once the object is held, call this function to move it along a given path
        :param cube: ID of Object to move
        :param data: Path points to move along
        :return: done: True if complete, False otherwise
        """
        marker_left_link = Markers.Marker()
        marker_right_link = Markers.Marker()
        marker_left_cp = Markers.Marker(color=[0,1,0,1])
        marker_right_cp = Markers.Marker(color=[0,1,0,1])
        marker_cube = Markers.Marker(color=[0,0,1,1])
        marker_human_data = Markers.Marker(color=[1,1,0,1])
        j = 0
        for line in data:
            # if j == 1:
            #     break
            print("ITERATION: {}".format(j))
            if j < 65:
                j += 1
                continue
            [next_contact_pose_left, next_contact_pose_right], [next_link_pose_left, next_link_pose_right], next_cube_pose = self.get_pose_in_world_origin(cube, line)
            # next_contact_points = [next_contact_pose_left[0], next_contact_pose_right[0]]
            next_link_pose = [next_link_pose_left[0], next_link_pose_right[0]]
            # print("NEXT CONTACT POINTS ARE: {}".format(next_contact_points))
            next_joint_poses = p.calculateInverseKinematics2(bodyUniqueId=self.gripper_id,
                                                             endEffectorLinkIndices=[self.joint_dict['L_Dist'],
                                                                                     self.joint_dict['R_Dist']],
                                                             targetPositions=next_link_pose)
            self.move_fingers_to_pose(next_joint_poses, abs_tol=1e-0)

            marker_pose_left_pos = list(next_link_pose_left[0])
            marker_pose_left_orn = list(next_link_pose_left[1])
            marker_pose_left_pos[2] = 0.06
            marker_pose_right_pos = list(next_link_pose_right[0])
            marker_pose_right_orn = list(next_link_pose_right[1])
            marker_pose_right_pos[2] = 0.06
            marker_left_link.set_marker_pose((marker_pose_left_pos, marker_pose_left_orn))
            marker_right_link.set_marker_pose((marker_pose_right_pos, marker_pose_right_orn))

            marker_pose_left_pos = list(next_contact_pose_left[0])
            marker_pose_left_orn = list(next_contact_pose_left[1])
            marker_pose_left_pos[2] = 0.08
            marker_pose_right_pos = list(next_contact_pose_right[0])
            marker_pose_right_orn = list(next_contact_pose_right[1])
            marker_pose_right_pos[2] = 0.08
            marker_left_cp.set_marker_pose((marker_pose_left_pos, marker_pose_left_orn))
            marker_right_cp.set_marker_pose((marker_pose_right_pos, marker_pose_right_orn))

            marker_cube_pos = list(next_cube_pose[0])
            marker_cube_orn = list(next_cube_pose[1])
            marker_cube_pos[2] = 0.08
            marker_cube.set_marker_pose((marker_cube_pos, marker_cube_orn))

            marker_next_pose = list(cube.next_pos)
            marker_next_pose[2] = 0.08
            marker_human_data.set_marker_pose((marker_next_pose, cube.next_orn))

            j += 1

        # print("CURRENT CONTACT POINTS ARE: {}".format(curr_contact_points))
        # Get Contact Points btw Cube and End Effectors
        # Build matrix btw c_p and cube
