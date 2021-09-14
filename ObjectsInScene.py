#!/usr/bin/env python3

import pybullet as p
# import setup
import numpy as np
from math import isclose, radians
import Markers
import time


class SceneObject:
    """
    SceneObject class
    To create an object in the simulator
    """

    def __init__(self, object_id):
        """
        Constructor
        :param gripper_id: SceneObject ID derived from loading object to the scene
        """
        self.object_id = object_id
        self.curr_pos, self.curr_orn = None, None
        self.next_pos = None
        self.next_orn = None
        self.start_pos, self.start_orn = p.getBasePositionAndOrientation(self.object_id)
        self.cube_size = 0.04
        # TODO: change this to reflect number of contact points
        self.start_pos_left, self.start_orn_left = (self.start_pos[0] - self.cube_size,
                                                    self.start_pos[1] - self.cube_size,
                                                    self.start_pos[2]), self.start_orn
        self.start_pos_right, self.start_orn_right = (self.start_pos[0] - self.cube_size,
                                                    self.start_pos[1] + self.cube_size,
                                                    self.start_pos[2]), self.start_orn
        self.start_cp = [(self.start_pos_left, self.start_orn_left), (self.start_pos_right, self.start_orn_right)]
        self.start_cube_start_cp = []
        self.start_pos_cube_origin, self.start_orn_cube_origin = p.invertTransform(self.start_pos, self.start_orn)
        for index in range(0, len(self.start_cp)):
            self.start_cube_start_cp.append(p.multiplyTransforms(self.start_pos_cube_origin, self.start_orn_cube_origin,
                                                                                self.start_cp[index][0],
                                                                                self.start_cp[index][1]))

    def get_curr_pose(self):
        self.curr_pos, self.curr_orn = p.getBasePositionAndOrientation(self.object_id)
        return self.curr_pos, self.curr_orn

    # def get_start_pose(self):
    #     self.start_pos, self.start_orn = p.getBasePositionAndOrientation(self.object_id)

    def get_next_pose(self, data, scale=0.001):
        """
        TODO: Change name to better represent what's happening here "convert_data_to_pose"?
        Get the next pose values from the file in the proper pose format
        :param data: Data line from file as a list [x, y, rotx,  f_x,f_y,f_rot_mag]
        :return:
        """
        # # Normalized data
        # self.next_pos = (data[3] * scale, 0.0, data[4] * scale)
        # orn_eul = [0, radians(data[2]), 0]

        # Non -normalized data
        # print(data[0], data[1], data[2])
        self.next_pos = (data[4] * scale, 0.0, data[5] * scale)
        orn_eul = [0, radians(data[6]), 0]

        self.next_orn = p.getQuaternionFromEuler(orn_eul)


if __name__ == "__main__":
    pass