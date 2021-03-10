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
        self.args = kwargs['args']
        self.action_space_type = kwargs['action_space_type']
        self.human_data = setup.read_file(self.args.path_to_human_data)
        if "Yes" in self.args.plot_human_data:
            print("PLOT? {}".format(self.args.plot_human_data))
            plot.plot_human_data(self.human_data)
        (self.physicsClient, self.planeID, self.num_objects, self.gripperID, self.objectIDs) = setup.init_sim(self.args.path_to_gripper_sdf)
        self.objectID = self.objectIDs[0]
        setup.set_camera_view(self.args.camera_view)
        self.gripper = Manipulator.Manipulator(self.gripperID, self.args.open_fingers_pose, self.args.start_grasp_pose)
        self.cube = ObjectsInScene.SceneObject(self.objectID)

        # Define  action space,  state space and reward

        if kwargs['state_space_type'] is None:
            self.observation_space = spaces.Box(np.array([0,0,0]), np.array([0,0,0]))

        if self.action_space_type == 'PATH':
            self.action_space = spaces.Discrete(4)

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
        """
        Takes a step in pybullet env
        :param action: should be joint angle changes as a list type
        :return:
            observation (object): agent's observation of the current environment
            reward (float) : amount of reward returned after previous action
            done (bool): whether the episode has ended, in which case further step() calls will return undefined results
            info (dict): contains auxiliary diagnostic information (helpful for debugging, and sometimes learning)

        """

        if self.action_space_type == 'PATH':
            pass
        # Take a step
        self.gripper.step_sim(action)
        # Get reward

        # Get the new observation
        return [np.array([0,0,0]), np.array([0,0,0])], 5, False, 5

    def render(self, mode='human'):
        pass

    def close(self):
        pass

    def seed(self, seed=None):
        pass


if __name__ == "__main__":
    pass
