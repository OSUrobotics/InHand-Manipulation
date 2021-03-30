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
import json


class IHMBulletEnv(gym.Env):

    def __init__(self, kwargs):
        # Initial arguments and setup
        self.episode_max_steps = 66
        self.episode_count = 0
        self.args = kwargs['args']
        self.human_data = setup.read_file(self.args.path_to_human_data)
        if "Yes" in self.args.plot_human_data:
            print("PLOT? {}".format(self.args.plot_human_data))
            plot.plot_human_data(self.human_data)
        (self.physicsClient, self.planeID, self.num_objects, self.gripperID, self.objectIDs) = setup.init_sim(
            self.args.path_to_gripper_sdf)
        self.objectID = self.objectIDs[0]
        setup.set_camera_view(self.args.camera_view)
        self.gripper = Manipulator.Manipulator(self.gripperID, self.args.open_fingers_pose, self.args.start_grasp_pose)
        self.cube = ObjectsInScene.SceneObject(self.objectID)

        # Define state space and reward
        print("JOINTS:", self.gripper.num_joints)
        # Read from JSON file
        with open('gym_env_files/envs/rl_spaces.json', 'r') as f:
            rl_spaces = json.load(f)
        print("LOOK HERE:", type(rl_spaces), rl_spaces)
        state_space = rl_spaces['state_space']
        self.obs_space = {}
        # Define State Space
        for key, value in state_space.items():
            # TODO: Change this to something more meaningful than IndexError
            try:
                if 'prox' in key:
                    shape = len(self.gripper.prox_indices)
                elif 'dist' in key:
                    shape = len(self.gripper.end_effector_indices)
                else:
                    shape = 2
                self.obs_space.update({key: spaces.Box(low=value[0], high=value[1], shape=(shape,))})
            except IndexError:
                self.obs_space.update({key: spaces.Box(low=-np.inf, high=np.inf, shape=(value[0],))})

        self.observation_space = spaces.Dict(self.obs_space)
        print(self.obs_space,"\nHERE:", self.observation_space)

        # Define Action Space
        action_space = rl_spaces['action_space']
        self.act_space = {}
        for key, value in action_space.items():
            self.act_space.update({key: spaces.Box(low=value[0], high=value[1], shape=(self.gripper.num_joints - 1,))})

        self.action_space = spaces.Dict(self.act_space)
        print("ACTION:", self.act_space, "\nHERE:", self.action_space)

        # Define rewards
        self.rewards = rl_spaces['reward']

        # Get full Spaces
        with open('gym_env_files/envs/all_options.json', 'r') as f:
            full_space = json.load(f)

        self.full_state_space = full_space['full_state_space']
        self.full_action_space = full_space['full_action_space']
        self.full_reward = full_space['full_reward']

    def reset(self):
        """
        Should  reset  object  pose  and   robot fingers  back to starting position
        :return:
        """
        # Move  hand to  open pose
        done_open, _ = self.gripper.move_fingers_to_pose(self.gripper.open_fingers_pose, abs_tol=0.1)
        print("Complete Open? {}".format(done_open))

        # Move object  to start pose
        p.resetBasePositionAndOrientation(self.cube.object_id, self.cube.start_pos, self.cube.start_orn)

        # Move hand to start grasp  pose
        done_grasp, contact_points = self.gripper.move_fingers_to_pose(self.gripper.start_grasp_pose, self.cube,
                                                                       abs_tol=0.05)
        print("Complete Grasp Object? {}, Contact  points: {}".format(done_grasp, contact_points))
        self.obs = self._get_next_observation()
        self.prev_cube_pos = self.obs['object_pos']
        self.prev_cube_orn = self.obs['object_orientation']
        self.episode_count = 0
        return self.obs

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
        # Select action type
        self.prev_cube_pos = self.obs['object_pos']
        self.prev_cube_orn = self.obs['object_orientation']
        # Take a step
        print("RANDOM ACTION:", list(action['joint_angles']))
        self.gripper.step_sim(list(action['joint_angles']))

        # Get the new observation
        self.obs = self._get_next_observation()

        # Get reward
        reward = self._get_next_reward()

        # Get done
        done = self._get_done_bit()

        # Get info
        info = None
        self.episode_count += 1

        return self.obs, reward, done, info

    def _get_next_observation(self):
        # pass through full list, if observation in list, update it
        self.gripper.get_joint_angles()
        self.cube.get_curr_pose()
        print(self.gripper.curr_joint_states)
        print(self.gripper.curr_joint_poses)
        print(self.gripper.prox_indices)
        next_obs = {}

        for key in dict(self.obs_space).keys():
            if key == 'joint_angles_prox':
                next_list = []
                for prox_index in self.gripper.prox_indices:
                    next_list.append(self.gripper.curr_joint_poses[prox_index-1])
                next_obs.update({key: next_list})

            elif key == 'joint_velocities_prox':
                next_list = []
                for prox_index in self.gripper.prox_indices:
                    next_list.append(self.gripper.curr_joint_states[prox_index-1][1])
                next_obs.update({key: next_list})

            elif key == 'joint_angles_dist':
                next_list = []
                for dist_index in self.gripper.end_effector_indices:
                    next_list.append(self.gripper.curr_joint_poses[dist_index-1])
                next_obs.update({key: next_list})

            elif key == 'joint_velocities_dist':
                next_list = []
                for dist_index in self.gripper.end_effector_indices:
                    next_list.append(self.gripper.curr_joint_states[dist_index-1][1])
                next_obs.update({key: next_list})

            elif key == 'object_pos':
                next_obs.update({key: self.cube.curr_pos})

            elif key == 'object_orientation':
                next_obs.update({key: self.cube.curr_orn})

            else:
                print("Key isn't part of full observation space")
                raise KeyError

        # print ("OBS:", next_obs)
        return next_obs

    def _get_next_reward(self):
        reward_dict = {}

        for reward_key in self.rewards.keys():
            # contact reward
            if reward_key == 'contact_reward':
                is_contact = self.gripper.get_contact_points(self.cube.object_id)
                if None in is_contact:
                    reward_value = self.rewards[reward_key][0]
                else:
                    reward_value = self.rewards[reward_key][1]

            # maintain path reward
            elif reward_key == 'path_reward':
                self.cube.get_curr_pose()
                target_pos, target_orn = self.sample_target_pose()
                if not np.isclose(self.cube.curr_pos, target_pos).all():
                    reward_value = self.rewards[reward_key][0]
                else:
                    reward_value = self.rewards[reward_key][1]

            else:
                print("REWARD not in full reward description")
                raise KeyError

            reward_dict.update({reward_key: reward_value})

        # print ("ALL REWARDS HERE:", reward_dict, "\n","VALUES:", sum(reward_dict.values()))
        reward = sum(reward_dict.values())
        return reward

    def sample_target_pose(self):
        line = np.random.randint(0, self.human_data.shape[0])
        (target_pos, target_orn) = self.gripper.get_origin_cube(self.cube, self.human_data[line])
        return target_pos,  target_orn

    def _get_done_bit(self):
        # If max episode count reached, raise done flag
        if self.episode_count >= self.episode_max_steps:
            done = True
            return done

        # If object stops moving, raise done flag
        if np.isclose(self.prev_cube_pos, self.obs['object_pos']).all() and np.isclose(self.prev_cube_orn,
                                                                                       self.obs['object_orientation'])\
                .all():
            done = True
            return done
        done = False
        return done

    def render(self, mode='human'):
        pass

    def close(self):
        pass

    def seed(self, seed=None):
        pass


if __name__ == "__main__":
    pass
