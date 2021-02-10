#!/usr/bin/env python3

import pybullet as p


class Marker:
    """
    A Visualization tool to help find particular points in the simulation
    """

    def __init__(self, shape=p.GEOM_SPHERE):
        """
        Initialising a marker with a shape
        :param shape: Shape of visual object being vreated
        """
        self.ID = p.createVisualShape(shapeType=shape)

    def set_marker_pose(self, pose):
        """
        Place marker at a given position and orientation
        :param pose: List containing pos and orn [(x, y, z), (r_x, r_y, r_z, w)] we want to place marker at.
        """
        p.createMultiBody(baseVisualShapeIndex=self.ID, basePosition=pose[0], baseOrientation=pose[1])
