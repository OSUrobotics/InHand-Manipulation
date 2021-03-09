import gym
from gym import spaces
import pybullet_envs
import time
import pybullet as p
import pybullet_data
import os, sys
import numpy as np
import setup
import ObjectsInScene
import Manipulator
import plot


class IHMBulletEnv(gym.Env):

    def __init__(self, kwargs):
        # Initial arguments and setup
        self.human_data = setup.read_file(kwargs.path_to_human_data)
        if "Yes" in kwargs.plot_human_data:
            print("PLOT? {}".format(kwargs.plot_human_data))
            plot.plot_human_data(self.human_data)
        (self.physicsClient, self.planeID, self.num_objects, self.gripperID, self.objectIDs) = setup.init_sim(kwargs.path_to_gripper_sdf)
        self.objectID = self.objectIDs[0]
        setup.set_camera_view(kwargs.camera_view)
        self.gripper = Manipulator.Manipulator(self.gripperID, kwargs.open_fingers_pose, kwargs.start_grasp_pose)
        self.cube = ObjectsInScene.SceneObject(self.objectID)

        self.action_space = spaces.Discrete(4)
        self.observation_space = spaces.Box(np.array([0,0,0]), np.array([0,0,0]))
        self.joint_indices = [1,2,3,4]

    def reset(self):
        """
        Should  reset  object  pose  and   robot fingers  back to starting position
        :return:
        """
        # Move object  to start pose
        p.resetBasePositionAndOrientation(self.cube.object_id, self.cube.start_pos, self.cube.start_orn)

        # Move  hand to  open pose
        done_open, _ = self.gripper.move_fingers_to_pose(self.gripper.open_fingers_pose, abs_tol=0.1)
        print("Complete Open? {}".format(done_open))

        # Move hand to start grasp  pose
        done_grasp, contact_points = self.gripper.move_fingers_to_pose(self.gripper.start_grasp_pose, self.cube, abs_tol=0.05)
        print("Complete Grasp Object? {}, Contact  points: {}".format(done_grasp, contact_points))

    def step(self, action):
        self.gripper.step_sim(action)
        return [np.array([0,0,0]), np.array([0,0,0])], 5, False, 5

    def render(self, mode='human'):
        pass

    def close(self):
        pass

    def seed(self, seed=None):
        pass


if __name__ == "__main__":
    pass
