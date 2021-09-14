#!/usr/bin/env python3

import pybullet as p
import setup
import numpy as np
import csv
import pandas as pd
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
        self.next_info = None
        self.next_joint_poses = None
        self.action_type = self.action_type_JA
        # self.target_marker = Markers.Marker(color=[0, 1, 0]).set_marker_pose([0, 0, 0])
        # self.cp_marker = Markers.Marker().set_marker_pose([0, 0, 0])
        self.object_traj_data = []
        self.phase = None
        self.human_data_file_name = ''
        self.human_data = None
        self.save_data_dict = {}
        self.only_first_entry = False
        self.cube_subtract_from_pos = None
        self.hand_type = None

        # Tunable parameters
        self.set_pid = False
        self.k_p = None
        self.k_d = None
        self.ep_step = 5
        self.limit_data = 10

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
                contact_points.append(contact_points_info[0][6])
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
                self.step_sim(pose_for_contact, cube)

            else:
                in_contact = True
                return in_contact
        raise EnvironmentError

    def action_type_JA(self, action, cube):
        # print("ACTION: {}, JOINTS: {}".format(action, self.joint_indices))
        if not self.set_pid:
            p.setJointMotorControlArray(bodyUniqueId=self.gripper_id, jointIndices=self.joint_indices,
                                        controlMode=p.POSITION_CONTROL, targetPositions=action)
        else:
            if self.k_p is not None and self.k_d is None:
                p.setJointMotorControlArray(bodyUniqueId=self.gripper_id, jointIndices=self.joint_indices,
                                            controlMode=p.POSITION_CONTROL, targetPositions=action,
                                            positionGains=self.k_p)
            elif self.k_p is None and self.k_d is not None:
                p.setJointMotorControlArray(bodyUniqueId=self.gripper_id, jointIndices=self.joint_indices,
                                            controlMode=p.POSITION_CONTROL, targetPositions=action,
                                            velocityGains=self.k_d)
            elif self.k_p is not None and self.k_d is not None:
                p.setJointMotorControlArray(bodyUniqueId=self.gripper_id, jointIndices=self.joint_indices,
                                            controlMode=p.POSITION_CONTROL, targetPositions=action,
                                            positionGains=self.k_p, velocityGains=self.k_d)
            else:
                print("###!!! Forgot to set pid values!!!###")
                raise ValueError

        self.get_joint_angles()
        # print(self.curr_joint_poses)

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

        for i in range(0, self.ep_step):
            p.stepSimulation()
            time.sleep(1. / 240.)
            self.action_type(move_to_joint_poses, cube)
        # if cube is not None:
        self.save_data(cube)

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
            # Query joint angles and check if they match or are close to where we want them to be
            done = self.pose_reached(joint_poses, abs_tol)
            if contact_check:
                # check for  contact
                try:
                    in_contact = self.check_for_contact(cube)
                    print("IN CONTACT? {}".format(in_contact))
                    self.step_sim(joint_poses, cube)

                except EnvironmentError:
                    print("We Have Lost Contact with Object. Aborting Mission")
                    setup.quit_sim()

            else:
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
        # print("NEXT POSE: {}".format(T_origin_nextpose_cube))
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
        T_origin_no_cp = p.multiplyTransforms(T_origin_target[0], T_origin_target[1], cube.start_cube_start_cp[index][0]
                                              , cube.start_cube_start_cp[index][1])
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
                T_origin_new_cp = self.get_origin_cp(i, cube, T_cube_origin, T_origin_nextpose_cube,
                                                     curr_contact_points)
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
        l_dist_marker = Markers.Marker()
        # l_dist_marker.set_marker_pose(p.getLinkState(self.gripper_id, 1)[4])
        # distal_pos = link[4])
        for line in data:
            print("ITERATION: {}".format(j))
            if j % self.limit_data != 0:
                j += 1
                continue
            if contact_check:
                try:
                    in_contact = self.check_for_contact(cube)
                except EnvironmentError:
                    print("We Have Lost Contact with Object. Aborting Mission")
                    self.save_file()
                    # setup.quit_sim()
                    break
            self.next_info = self.get_pose_in_world_origin_expert(cube, line)
            self.next_joint_poses = p.calculateInverseKinematics2(bodyUniqueId=self.gripper_id,
                                                                  endEffectorLinkIndices=self.end_effector_indices,
                                                                  targetPositions=self.next_info[2])
            # l_dist_marker.set_marker_pose(self.next_info[0][0])
            self.move_fingers_to_pose(self.next_joint_poses, cube, abs_tol=1e-0, contact_check=False)

            j += 1

        self.save_file()

        # # To set Marker pose -> something like this
        # for i in range(0, len(next_info)):
        #     for pos in next_info[i]:
        #         Markers.Marker(color=[1, 0, 0]).set_marker_pose(pos)

    def save_human_data(self, cube):
        """
        TODO: fix this to read from file!
        :param cube:
        :return:
        """
        if self.phase is 'Move':
            pos = cube.next_pos
            quat_orn = cube.next_orn
            if quat_orn is not None:
                orn = p.getEulerFromQuaternion(quat_orn)
            else:
                orn = None
        else:
            pos = None
            orn = None
        self.add_column(column_name='human_cube_pos', column_data=pos)
        self.add_column(column_name='human_cube_orn', column_data=orn)

    def save_cube_data(self, cube):
        # print("CUBE? {}:".format(cube))
        if cube is not None:
            cube_pose = cube.get_curr_pose()
            cube_pos = cube_pose[0]
            cube_orn = p.getEulerFromQuaternion(cube_pose[1])

            if not self.only_first_entry and self.phase is 'Move':
                self.cube_subtract_from_pos = cube_pose[0]
                self.only_first_entry = True

        else:
            cube_pos, cube_orn = None, None

        self.add_column(column_name='Cube_Pos', column_data=cube_pos)
        self.add_column(column_name='Cube_Orn', column_data=cube_orn)

    def save_links_data(self):
        links_info = p.getLinkStates(self.gripper_id, linkIndices=list(self.joint_dict.values()))
        for i in range(len(self.key_names_list)):
            link_pos_x = links_info[i][4][0]
            link_pos_y = links_info[i][4][1]
            link_pos_z = links_info[i][4][2]
            link_orn = p.getEulerFromQuaternion(links_info[i][5])
            link_orn_x = link_orn[0]
            link_orn_y = link_orn[1]
            link_orn_z = link_orn[2]
            self.add_column(column_name='{}_{}'.format(self.key_names_list[i], 'link_pos'), column_data=[link_pos_x,
                                                                                                         link_pos_y,
                                                                                                         link_pos_z])
            self.add_column(column_name='{}_{}'.format(self.key_names_list[i], 'link_orn'), column_data=[link_orn_x,
                                                                                                         link_orn_y,
                                                                                                         link_orn_z])

    def save_joints_data(self):
        joints_info = p.getJointStates(self.gripper_id, jointIndices=list(self.joint_dict.values()))
        for i in range(len(self.key_names_list)):
            joint_angle = joints_info[i][0]
            joint_vel = joints_info[i][1]
            joint_react_forces = joints_info[i][2]
            joint_motor_torque = joints_info[i][3]
            self.add_column(column_name='{}_{}'.format(self.key_names_list[i], 'joint_angle'), column_data=joint_angle)
            self.add_column(column_name='{}_{}'.format(self.key_names_list[i], 'joint_vel'), column_data=joint_vel)
            self.add_column(column_name='{}_{}'.format(self.key_names_list[i], 'joint_react_forces'),
                            column_data=joint_react_forces)
            self.add_column(column_name='{}_{}'.format(self.key_names_list[i], 'joint_motor_torque'),
                            column_data=joint_motor_torque)

    def save_contact_data(self, cube):
        """
        TODO: Potential problem when more that two fingers
        :param cube:
        :return:
        """
        contact_points_info = []
        data = []
        in_contact = []
        if cube is not None:
            for i in range(0, len(self.end_effector_indices)):
                contact_points_info.append(p.getContactPoints(cube.object_id, self.gripper_id,
                                                              linkIndexB=self.end_effector_indices[i]))
                if i == 0:
                    start, end = 0, 4
                elif i == 1:
                    start, end = 4, 8
                else:
                    raise IndexError

                if len(contact_points_info[i]) > 0:
                    for j in range(6, 10):
                        data.append(contact_points_info[i][0][j])
                    in_contact.append(True)

                else:
                    for _ in range(start, end):
                        data.append(None)
                    in_contact.append(False)

        else:
            for _ in range(0, 8):
                data.append(None)
            in_contact = [False, False]

        self.add_column(column_name='in_contact_l', column_data=in_contact[0])
        self.add_column(column_name='contact_point_l_dist', column_data=data[0])
        self.add_column(column_name='contact_normal_l_dist', column_data=data[1])
        self.add_column(column_name='contact_distance_l_dist', column_data=data[2])
        self.add_column(column_name='contact_norm_force_l_dist', column_data=data[3])

        self.add_column(column_name='in_contact_r', column_data=in_contact[1])
        self.add_column(column_name='contact_point_r_dist', column_data=data[4])
        self.add_column(column_name='contact_normal_r_dist', column_data=data[5])
        self.add_column(column_name='contact_distance_r_dist', column_data=data[6])
        self.add_column(column_name='contact_norm_force_r_dist', column_data=data[7])

    def get_cube_in_start_pos(self, cube):
        if self.phase is 'Move' and self.cube_subtract_from_pos is not None:
            cube_curr_pos = cube.get_curr_pose()[0]
            new_pos = [cube_curr_pos[0] - self.cube_subtract_from_pos[0],
                       cube_curr_pos[1] - self.cube_subtract_from_pos[1],
                       cube_curr_pos[2] - self.cube_subtract_from_pos[2]]
        else:
            new_pos = None
        self.add_column(column_name='Cube_pos_in_start_pos', column_data=new_pos)

    def save_data(self, cube):
        self.add_column(column_name='Phase', column_data=self.phase)
        self.save_human_data(cube)
        self.save_cube_data(cube)
        self.get_cube_in_start_pos(cube)
        self.save_links_data()
        self.save_joints_data()
        self.save_contact_data(cube)

    def add_column(self, column_name='', column_data=None):
        if column_name not in self.save_data_dict.keys():
            self.save_data_dict.update({column_name: [column_data]})
        else:
            self.save_data_dict[column_name].append(column_data)
        pass

    def save_file(self):
        print("DICT: {}".format(self.save_data_dict))
        df = pd.DataFrame.from_dict(self.save_data_dict)
        print("DF {}".format(df.items))
        print(self.hand_type)
        df.to_csv('AnalyseData/Data/Trial Data/Tries/{}_'.format(self.hand_type) + self.human_data_file_name + '_kp{}_kd{}_dp{}_step{}'.format(self.k_p, self.k_d, self.limit_data, self.ep_step) +'_save_data.csv')


if __name__ == "__main__":
    """
    Human Data: cube position in cms
    Expert Data:
    Cube position: 
    """
    pass
