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
        self.start_pos, self.start_orn = self.get_curr_pose()

    def get_curr_pose(self):
        self.curr_pos, self.curr_orn = p.getBasePositionAndOrientation(self.object_id)
        return self.curr_pos, self.curr_orn

    # def get_start_pose(self):
    #     self.start_pos, self.start_orn = p.getBasePositionAndOrientation(self.object_id)

    def get_next_pose(self, data, scale=0.1):
        """
        Get the next pose values from the file in the proper pose format
        :param data: Data line from file as a list [x, y, rotx,  f_x,f_y,f_rot_mag]
        :return:
        """
        # self.next_pos = (data[1] * scale, data[0] * scale, self.curr_pos[2])
        self.next_pos = (-data[1]*scale, 0.0, -data[0]*scale)
        orn_eul = [0, radians(data[2]), 0]
        self.next_orn = p.getQuaternionFromEuler(orn_eul)
        # orn = ((1, 1, 0), radians(data[2]))
        # self.next_orn = p.getQuaternionFromAxisAngle(orn[0], orn[1])


if __name__ == "__main__":
    pass