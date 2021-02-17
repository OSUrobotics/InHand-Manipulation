#!/usr/bin/env python3

import pybullet as p
import setup
import numpy as np
from math import isclose, radians
import Markers
import time
from ObjectsInScene import SceneObject


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

    # def __repr__(self):
    #     return "This Gripper has {} joints called {}".format(self.num_joints, self.joint_dict_with_base.keys())

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

    def check_for_contact(self, cube):
        """
        MAKES SURE CUBE IS IN contact with  hand
        :param cube: cube object from objectsinscene class
        :return:
        """
        tot_attempts = 20
        in_contact = False
        for i in range(0, tot_attempts):
            p.stepSimulation()
            time.sleep(1. / 240.)
            [contact_points_left, contact_points_right] = self.get_contact_points(cube.object_id)
            if contact_points_left is None or contact_points_right is None:
                # maintain contact
                go_to = cube.get_curr_pose()[0]
                print("Attempting to make contact")
                pose_for_contact = p.calculateInverseKinematics2(bodyUniqueId=self.gripper_id,
                                                                 endEffectorLinkIndices=[self.joint_dict['L_Dist'],
                                                                                         self.joint_dict['R_Dist']],
                                                                 targetPositions=[go_to, go_to])
                p.setJointMotorControlArray(bodyIndex=self.gripper_id, jointIndices=self.joint_indices,
                                            controlMode=p.POSITION_CONTROL, targetPositions=pose_for_contact)

            else:
                in_contact = True
                return in_contact
        raise EnvironmentError

    def move_fingers_to_pose(self, joint_poses, cube=None, abs_tol=1e-05, contact_check=False):
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
        # original_distal_left = Markers.Marker()
        left_link = p.getLinkState(self.gripper_id, self.joint_dict['L_Dist'])
        distal_left_pos = left_link[4]
        distal_left_orn = left_link[5]
        # original_distal_left.set_marker_pose((distal_left_pos, distal_left_orn))

        while p.isConnected() and not done:
            p.stepSimulation()
            time.sleep(1. / 240.)

            # targetVelocities=v, forces=f)
            # Query joint angles and check if they match or are close to where we want them to be
            done = self.pose_reached(joint_poses, abs_tol)
            if contact_check:
                # check for  contact
                try:
                    in_contact = self.check_for_contact(cube)
                    print("IN CONTACT? {}".format(in_contact))
                    p.setJointMotorControlArray(bodyIndex=self.gripper_id, jointIndices=self.joint_indices,
                                                controlMode=p.POSITION_CONTROL, targetPositions=joint_poses)

                except EnvironmentError:
                    print("We Have Lost Contact with Object. Aborting Mission")
                    setup.quit_sim()

            else:
                p.setJointMotorControlArray(bodyIndex=self.gripper_id, jointIndices=self.joint_indices,
                                            controlMode=p.POSITION_CONTROL, targetPositions=joint_poses)
            # time.sleep(1)
        if cube is not None:
            contact_points = self.get_contact_points(cube.object_id)

        else:
            contact_points = [None, None]

        return done, contact_points

    def get_origin_cube(self, cube, data):
        """
        Gets the next pose of the  cube in world coordinates
        :param cube:
        :param data:
        :return:
        """
        cube.get_next_pose(data)
        T_origin_next_pose_cube = p.multiplyTransforms(cube.start_pos, cube.start_orn, cube.next_pos, cube.next_orn)
        # print("Start: {} {},\n Next: {} {}, \n Next in Origin: {} ".format(cube.start_pos,
        #                                                                    p.getEulerFromQuaternion(cube.start_orn),
        #                                                                    cube.next_pos,
        #                                                                    p.getEulerFromQuaternion(cube.next_orn),
        #                                                                    T_origin_next_pose_cube[0]))
        # marker_pose = [list(T_origin_next_pose_cube[0]), list(T_origin_next_pose_cube[1])]
        # marker_pose[0][2] = 0.12
        # Markers.Marker(marker_pose)
        return T_origin_next_pose_cube

    def get_origin_cp(self, i, cube, T_cube_origin, T_origin_nextpose_cube, curr_contact_points):
        """

        :return:
        """
        T_cube_cp = p.multiplyTransforms(T_cube_origin[0], T_cube_origin[1], curr_contact_points[i], self.curr_orn)
        T_origin_new_cp = p.multiplyTransforms(T_origin_nextpose_cube[0], T_origin_nextpose_cube[1], T_cube_cp[0], T_cube_cp[1])

        # marker_pose = [list(T_origin_new_cp[0]), list(T_origin_new_cp[1])]
        # marker_pose[0][2] = 0.12
        # Markers.Marker(marker_pose, color=[0,1,0,1])

        return T_origin_new_cp

    def get_origin_links(self, i, j, T_origin_newcontactpoints,  curr_contact_points):
        """

        :param i:
        :param T_origin_newcontactpoints:
        :return:
        """
        T_cp_origin = p.invertTransform(curr_contact_points[i], self.curr_orn)
        left_link = p.getLinkState(self.gripper_id, j)
        distal_left_pos = left_link[4]
        distal_left_orn = left_link[5]
        T_cp_link = p.multiplyTransforms(T_cp_origin[0], T_cp_origin[1], distal_left_pos, distal_left_orn)
        T_origin_nl = p.multiplyTransforms(T_origin_newcontactpoints[i][0], T_origin_newcontactpoints[i][1],
                                           T_cp_link[0], T_cp_link[1])

        # marker_pose = [list(T_origin_nl[0]), list(T_origin_nl[1])]
        # marker_pose[0][2] = 0.12
        # Markers.Marker(marker_pose, color=[0,0,1,1])

        return T_origin_nl

    def get_pose_in_world_origin(self, cube, data):
        """
        Gets the new contact points in world coordinates
        :param cube: instance of object in scene class(object to move)
        :param data: line in file of human data [x,y,rmag,f_x,f_y,f_rot_mag]
        :return: list T_origin_newcontactpoints: next contact points in world coordinates for left and right
        """

        T_origin_nextpose_cube = self.get_origin_cube(cube, data)
        curr_contact_points = self.get_contact_points(cube.object_id)
        self.get_curr_pose()
        T_cube_origin = p.invertTransform(cube.curr_pos, cube.curr_orn)
        T_origin_newcontactpoints = []
        T_origin_newlink = []
        j = self.joint_dict['L_Dist']
        for i in range(0, len(curr_contact_points)):
            T_origin_new_cp = self.get_origin_cp(i, cube, T_cube_origin, T_origin_nextpose_cube, curr_contact_points)
            T_origin_newcontactpoints.append(T_origin_new_cp)
            T_origin_nl = self.get_origin_links(i, j, T_origin_newcontactpoints, curr_contact_points)
            T_origin_newlink.append(T_origin_nl)
            j += 2
        return T_origin_newcontactpoints, T_origin_newlink, T_origin_nextpose_cube

    def manipulate_object(self, cube, data, contact_check=False):
        """
        TODO: Dictionary keys should (maybe?) be a parameter?
        Once the object is held, call this function to move it along a given path
        :param cube: ID of Object to move
        :param data: Path points to move along
        :param contact_check: True if you want  to maintain contact with object
        :return: done: True if complete, False otherwise
        """
        marker_link_left = Markers.Marker(color=[1, 0, 0])
        marker_link_right = Markers.Marker(color=[1, 0, 0])
        marker_cp_left = Markers.Marker(color=[0, 1, 0], height=0.12)
        marker_cp_right = Markers.Marker(color=[0, 1, 0], height=0.12)
        marker_cube = Markers.Marker(color=[0,0,1], height=0.12)
        j = 0
        for line in data:
            # if j == 1:
            #     break
            print("ITERATION: {}".format(j))
            if j < 65:
                j += 1
                continue
            if contact_check:
                try:
                    in_contact = self.check_for_contact(cube)
                except EnvironmentError:
                    print("We Have Lost Contact with Object. Aborting Mission")
                    setup.quit_sim()

            [next_contact_pose_left, next_contact_pose_right], [next_link_pose_left, next_link_pose_right], \
            next_cube_pose = self.get_pose_in_world_origin(cube, line)
            next_contact_points = [next_contact_pose_left[0], next_contact_pose_right[0]]
            next_link_pose = [next_link_pose_left[0], next_link_pose_right[0]]
            next_joint_poses = p.calculateInverseKinematics2(bodyUniqueId=self.gripper_id,
                                                             endEffectorLinkIndices=[self.joint_dict['L_Dist'],
                                                                                     self.joint_dict['R_Dist']],
                                                             targetPositions=next_link_pose)
            self.move_fingers_to_pose(next_joint_poses, cube, abs_tol=1e-0,
                                      contact_check=False)

            # marker_link_left.set_marker_pose(next_link_pose[0])
            # marker_link_right.set_marker_pose(next_link_pose[1])

            # marker_cp_left.set_marker_pose(next_contact_points[0])
            # marker_cp_right.set_marker_pose(next_contact_points[1])

            # marker_cube.set_marker_pose(next_cube_pose[0])

            j += 1


if __name__ == "__main__":
    pass