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
        self.curr_joint_poses = []
        self.curr_joint_states = []
        self.gripper_id = gripper_id
        super(Manipulator, self).__init__(self.gripper_id)
        self.joint_info = []
        self.num_joints = p.getNumJoints(self.gripper_id)
        self.joint_indices = None
        self.open_fingers_pose = open_fingers_pose
        self.start_grasp_pose = start_grasp_pos
        self.joint_dict = {}
        self.joint_dict_with_base = {}
        self.key_names_list = []
        self.key_names_list_with_base = []
        self.end_effector_indices = []
        self.prev_cp = None
        self.prox_indices = []
        self.create_joint_dict(key_names)
        self.next_info= None
        self.next_joint_poses = None
        self.action_type = self.action_type_JA
        self.target_marker = Markers.Marker(color=[0, 1, 0]).set_marker_pose([0, 0, 0])
        self.cp_marker = Markers.Marker().set_marker_pose([0, 0, 0])
        self.object_traj_data = []


    # def __repr__(self):
    #     return "This Gripper has {} joints called {}".format(self.num_joints, self.joint_dict_with_base.keys())

    def create_joint_dict(self, keys):
        """
        Create a dictionary for easy referencing of joints
        :param keys: If None, uses nomenclature from urdf file, else uses nomenclature passed
        Note: 0 index is reserved for Base of the manipulator. Expected key for this is always 'Base'
        :return:
        """

        self.get_joints_info()
        for i in range(0, len(self.joint_info)):
            # if i == 0:
            #     self.joint_dict_with_base.update({self.joint_info[i][1]: self.joint_info[i][0]})
            #     self.key_names_list_with_base.append(self.joint_info[i][1])
            #     continue
            self.joint_dict.update({self.joint_info[i][1]: self.joint_info[i][0]})
            self.joint_dict_with_base.update({self.joint_info[i][1]: self.joint_info[i][0]})
            self.key_names_list.append(self.joint_info[i][1])
            self.key_names_list_with_base.append(self.joint_info[i][1])
            if 'dist' in str(self.joint_info[i][1]):
                self.end_effector_indices.append(i)

            if 'prox' in str(self.joint_info[i][1]):
                self.prox_indices.append(i)

        print(self.prox_indices)
        self.joint_indices = np.arange(self.prox_indices[0], self.num_joints)

        print("DICTIONARY CREATED:{}".format(self.joint_dict))
        print("DICTIONARY CREATED WITH BASE:{}".format(self.joint_dict_with_base))
        print("LIST CREATED:{}".format(self.key_names_list))
        print("LIST CREATED WITH BASE:{}".format(self.key_names_list_with_base))
        print("END EFFECTOR LIST CREATED:{}".format(self.end_effector_indices))

    def get_joints_info(self):
        """
        TODO: make this private?
        Get joint info of every joint of the manipulator
        :return: list of joint info of every joint
        """
        self.joint_info = []
        for joint in range(self.num_joints):
        # for joint in range(0, p.getNumJoints(self.gripper_id)):
            self.joint_info.append(p.getJointInfo(self.gripper_id, joint))
        print("Joint Info: {}".format(self.joint_info))
        return self.joint_info

    def get_joint_angles(self):
        """
        TODO: make this private?
        Get the current pose angle of joints
        Stores in self.curr_joint_angle : current joint angles as a list
        """
        self.curr_joint_poses = []
        self.curr_joint_states = p.getJointStates(self.gripper_id, self.joint_dict.values())
        for joint in range(0, len(self.curr_joint_states)):
            self.curr_joint_poses.append(self.curr_joint_states[joint][0])

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
        contact_points = []
        for i in range(0, len(self.end_effector_indices)):
            contact_points_info = p.getContactPoints(cube_id, self.gripper_id, linkIndexB=self.end_effector_indices[i])
            try:
                contact_points.append(contact_points_info[0][5])
            except IndexError:
                contact_points.append(None)
            pass

        return contact_points

    def check_for_contact(self, cube):
        """
        MAKES SURE CUBE IS IN contact with  hand
        :param cube: cube object from objectsinscene class
        :return:
        """
        tot_attempts = 30
        in_contact = False
        for i in range(0, tot_attempts):
            target_pos = []
            # p.stepSimulation()
            # time.sleep(1. / 240.)
            contact_points = self.get_contact_points(cube.object_id)
            if None in contact_points:
                # maintain contact
                go_to = cube.get_curr_pose()[0]
                for j in range(0, len(self.end_effector_indices)):
                    target_pos.append(go_to)
                print("Attempting to make contact")
                pose_for_contact = p.calculateInverseKinematics2(bodyUniqueId=self.gripper_id,
                                                                 endEffectorLinkIndices=self.end_effector_indices,
                                                                 targetPositions=target_pos)
                # p.setJointMotorControlArray(bodyIndex=self.gripper_id, jointIndices=self.joint_indices,
                #                             controlMode=p.POSITION_CONTROL, targetPositions=pose_for_contact)
                self.step_sim(pose_for_contact, cube)

            else:
                in_contact = True
                return in_contact
        raise EnvironmentError

    def action_type_JA(self, action, cube):
        # print("ACTION: {}, JOINTS: {}".format(action, self.joint_indices))
        p.setJointMotorControlArray(bodyUniqueId=self.gripper_id, jointIndices=self.joint_indices,
                                    controlMode=p.POSITION_CONTROL, targetPositions=action)
        self.get_joint_angles()
        print(self.curr_joint_poses)

    def action_type_JV(self, action, cube):
        p.setJointMotorControlArray(bodyUniqueId=self.gripper_id, jointIndices=self.joint_indices,
                                    controlMode=p.VELOCITY_CONTROL, targetVelocities=action)

    def action_type_PI(self, action, cube):
        action_dict = dict(action)
        action_values = action_dict['x_y']
        action_values = np.append(action_values, action_dict['deg_orn'])
        self.next_info = self.get_pose_in_world_origin(cube, action_values)

        self.next_joint_poses = p.calculateInverseKinematics2(bodyUniqueId=self.gripper_id,
                                                              endEffectorLinkIndices=self.end_effector_indices,
                                                              targetPositions=self.next_info[2])
        p.setJointMotorControlArray(bodyUniqueId=self.gripper_id, jointIndices=self.joint_indices,
                                    controlMode=p.POSITION_CONTROL, targetPositions=self.next_joint_poses)

    def step_sim(self, move_to_joint_poses, cube):
        """
        Take a step in the simulation
        :param move_to_joint_poses:
        :return:
        """
        p.stepSimulation()
        time.sleep(1. / 240.)
        self.action_type(move_to_joint_poses, cube)

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

        while p.isConnected() and not done:
            # p.stepSimulation()
            # time.sleep(1. / 240.)
            # Query joint angles and check if they match or are close to where we want them to be
            done = self.pose_reached(joint_poses, abs_tol)
            if contact_check:
                # check for  contact
                try:
                    in_contact = self.check_for_contact(cube)
                    print("IN CONTACT? {}".format(in_contact))
                    # p.setJointMotorControlArray(bodyIndex=self.gripper_id, jointIndices=self.joint_indices,
                    #                             controlMode=p.POSITION_CONTROL, targetPositions=joint_poses)

                    #Should we  be sending th ecube? @anjali possible error here?
                    self.step_sim(joint_poses, cube)

                except EnvironmentError:
                    print("We Have Lost Contact with Object. Aborting Mission")
                    setup.quit_sim()

            else:
                # p.setJointMotorControlArray(bodyIndex=self.gripper_id, jointIndices=self.joint_indices,
                #                             controlMode=p.POSITION_CONTROL, targetPositions=joint_poses)
                self.step_sim(joint_poses, cube)
        if cube is not None:
            contact_points = self.get_contact_points(cube.object_id)

        else:
            contact_points = [None, None]

        return done, contact_points

    def get_origin_cube(self, cube, data):
        """
        TODO: make this private?
        Gets the next pose of the  cube in world coordinates
        :param cube:
        :param data:
        :return:
        """
        cube.get_next_pose(data)
        T_origin_next_pose_cube = p.multiplyTransforms(cube.start_pos, cube.start_orn, cube.next_pos, cube.next_orn)
        return T_origin_next_pose_cube

    def get_origin_cp(self, i, cube, T_cube_origin, T_origin_nextpose_cube, curr_contact_points):
        """
        TODO: make this private?
        :return:
        """
        T_cube_cp = p.multiplyTransforms(T_cube_origin[0], T_cube_origin[1], curr_contact_points[i], self.curr_orn)
        T_origin_new_cp = p.multiplyTransforms(T_origin_nextpose_cube[0], T_origin_nextpose_cube[1],
                                           T_cube_cp[0], T_cube_cp[1])

        return T_origin_new_cp

    def get_origin_links(self, i, j, T_origin_newcontactpoints_pos, T_origin_newcontactpoints_orn, curr_contact_points):
        """
        TODO: make this private?
        :param i:
        :param T_origin_newcontactpoints:
        :return:
        """
        T_cp_origin = p.invertTransform(curr_contact_points[i], self.curr_orn)
        link = p.getLinkState(self.gripper_id, j)
        distal_pos = link[4]
        distal_orn = link[5]
        T_cp_link = p.multiplyTransforms(T_cp_origin[0], T_cp_origin[1], distal_pos, distal_orn)
        T_origin_nl = p.multiplyTransforms(T_origin_newcontactpoints_pos[i], T_origin_newcontactpoints_orn[i],
                                           T_cp_link[0], T_cp_link[1])

        return T_origin_nl

    def get_pose_in_world_origin_expert(self, cube, data):
        """
        TODO: make this private?
        Gets the new contact points in world coordinates
        :param cube: instance of object in scene class(object to move)
        :param data: line in file of human data [x,y,rmag,f_x,f_y,f_rot_mag]
        :return: list T_origin_newcontactpoints: next contact points in world coordinates for left and right
        """

        T_origin_nextpose_cube = self.get_origin_cube(cube, data)
        curr_contact_points = self.get_contact_points(cube.object_id)
        self.get_curr_pose()

        # TODO: Why does this not change anything?
        cube.get_curr_pose()

        T_cube_origin = p.invertTransform(cube.curr_pos, cube.curr_orn)
        T_origin_new_cp_pos = []
        T_origin_new_cp_orn = []
        T_origin_new_link_pos = []
        T_origin_new_link_orn = []
        for i in range(0, len(self.end_effector_indices)):
            T_origin_new_cp = self.get_origin_cp(i, cube, T_cube_origin, T_origin_nextpose_cube, curr_contact_points)
            T_origin_new_cp_pos.append(T_origin_new_cp[0])
            T_origin_new_cp_orn.append(T_origin_new_cp[1])
            T_origin_nl = self.get_origin_links(i, self.end_effector_indices[i], T_origin_new_cp_pos,
                                                T_origin_new_cp_orn, curr_contact_points)
            T_origin_new_link_pos.append(T_origin_nl[0])
            T_origin_new_link_orn.append(T_origin_nl[1])

        return [T_origin_new_cp_pos, T_origin_new_cp_orn, T_origin_new_link_pos, T_origin_new_link_orn,
                T_origin_nextpose_cube]

    def get_origin_no_cp_pts(self, cube, index, T_origin_target):
        T_origin_no_cp = p.multiplyTransforms(T_origin_target[0], T_origin_target[1], cube.start_cube_start_cp[index][0],
                                              cube.start_cube_start_cp[index][1])
        # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@!!!!!!!!!!!!!!!!!!!!!!!!@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        # Markers.Marker().reset_marker_pose(T_origin_no_cp[0], self.cp_marker)
        return T_origin_no_cp

    def get_pose_in_world_origin(self, cube, data):
        """
        Gets the new contact points in world coordinates
        :param cube: instance of object in scene class(object to move)
        :param data: line in file of human data [x,y,rmag,f_x,f_y,f_rot_mag]
        :return: list T_origin_newcontactpoints: next contact points in world coordinates for left and right
        """
        T_origin_nextpose_cube = self.get_origin_cube(cube, data)
        # Markers.Marker(color=[0,1,0]).reset_marker_pose(T_origin_nextpose_cube[0], self.target_marker)
        curr_contact_points = self.get_contact_points(cube.object_id)
        cube.get_curr_pose()
        self.get_curr_pose()
        T_cube_origin = p.invertTransform(cube.curr_pos, cube.curr_orn)
        T_origin_new_cp_pos = []
        T_origin_new_cp_orn = []
        T_origin_new_link_pos = []
        T_origin_new_link_orn = []
        for i in range(0, len(self.end_effector_indices)):
            if None in curr_contact_points:
                T_origin_new_cp = self.get_origin_no_cp_pts(cube, i, T_origin_nextpose_cube)
                T_origin_new_cp_pos.append(T_origin_new_cp[0])
                T_origin_new_cp_orn.append(T_origin_new_cp[1])
                T_origin_nl = T_origin_new_cp
            else:
                T_origin_new_cp = self.get_origin_cp(i, cube, T_cube_origin, T_origin_nextpose_cube, curr_contact_points)
                T_origin_new_cp_pos.append(T_origin_new_cp[0])
                T_origin_new_cp_orn.append(T_origin_new_cp[1])
                T_origin_nl = self.get_origin_links(i, self.end_effector_indices[i], T_origin_new_cp_pos,
                                                    T_origin_new_cp_orn, curr_contact_points)
            T_origin_new_link_pos.append(T_origin_nl[0])
            T_origin_new_link_orn.append(T_origin_nl[1])

        return [T_origin_new_cp_pos, T_origin_new_cp_orn, T_origin_new_link_pos, T_origin_new_link_orn,
                T_origin_nextpose_cube]

    def manipulate_object(self, cube, data, contact_check=False):
        """
        Once the object is held, call this function to move it along a given path
        :param cube: ID of Object to move
        :param data: Path points to move along
        :param contact_check: True if you want  to maintain contact with object
        :return: done: True if complete, False otherwise
        """
        j = 0
        for line in data:
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
            self.next_info = self.get_pose_in_world_origin_expert(cube, line)
            self.next_joint_poses = p.calculateInverseKinematics2(bodyUniqueId=self.gripper_id,
                                                             endEffectorLinkIndices=self.end_effector_indices,
                                                             targetPositions=self.next_info[2])
            self.move_fingers_to_pose(self.next_joint_poses, cube, abs_tol=1e-0,
                                      contact_check=False)
            self.save_object_traj(cube)
            j += 1

            # # To set Marker pose -> something like this
            # for i in range(0, len(next_info)):
            #     for pos in next_info[i]:
            #         Markers.Marker(color=[1, 0, 0]).set_marker_pose(pos)

    def save_object_traj(self, cube):
        """
        Save the x, y, z data of cube
        :return:
        """
        cube_curr_orn_in_eul = p.getEulerFromQuaternion(cube.curr_orn)
        cube_curr_pos_x = cube.curr_pos[1]
        cube_curr_pos_y = cube.curr_pos[0]
        self.object_traj_data.append([[cube_curr_pos_x, cube_curr_pos_y, cube.curr_pos[2]], [cube_curr_orn_in_eul[0],
                                                                                             cube_curr_orn_in_eul[1],
                                                                                             cube_curr_orn_in_eul[2]]])


if __name__ == "__main__":
    pass
