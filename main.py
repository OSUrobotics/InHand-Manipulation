#!/usr/bin/env python3
import setup
import ObjectsInScene
import pybullet as p
import time
import Manipulator
import plot
import argparse
import gym
import gym_env_files
import numpy as np
import gui
from PyQt5.QtWidgets import QApplication


def load_from_file(filename, parser, namespace):
    parser.add_argument("--path_to_human_data", default='Human Study Data/anjali_data_better/filt_josh_2v2_c_none_1.csv')
    parser.add_argument("--plot_human_data", default="No")
    parser.add_argument("--camera_view", default="TOP")
    parser.add_argument("--path_to_gripper_sdf", default='ExampleSimWorld-Josh/new/testing2_try.sdf')
    parser.add_argument("--open_fingers_pose", action='append', type=float)
    parser.add_argument("--start_grasp_pose", action='append', type=float)

    with open(filename, 'r') as f:
        for line in f:
            if "PASS ARGUMENTS HERE:" in line:
                continue
            else:
                parser.parse_args(line.strip().split('='), namespace)


def import_arguments(args, parser):
    if args.file is not None:
        with open('setup.txt', 'w') as f:
            f.write(args.file)

    with open('setup.txt', 'r') as f:
        filename = f.read()
    load_from_file(filename, parser, args)


if __name__ == "__main__":
    """
    Start GUI Interface
    """
    # GUI Stuff
    app = QApplication([])
    interface = gui.GUI()
    interface.show()
    app.exec_()


    """
    Parse Arguments and start sim
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', default=None)
    args = parser.parse_args()
    print("ARGUMENTS PASSED BEFORE: {}".format(args))
    import_arguments(args, parser)
    print("ARGUMENTS PASSED: {}".format(args))


    #  # Moving code
   #
   #  done_mov_obj = gripper.manipulate_object(cube, human_data, contact_check=True)
   #  time.sleep(2)

    """
    For importing gym env
    """
    env = gym.make("ihm-v0", kwargs={'args': args})
    env.reset()
    done_mov_obj = env.gripper.manipulate_object(env.cube, env.human_data, contact_check=True)
    # i = 0
    # # action = np.array([0.17, 0.0, -0.17, 0.0])
    # while i < 3000:
    #     # print(i)
    #     obs, reward, done, info = env.step(action=env.action_space.sample())
    #     i += 1
    #     print("EPISODE COUNT: {}, DONE BIT: {}, REWARD: {}, ITER: {}".format(env.episode_count, done, reward, i))
    #     if done:
    #         obs = env.reset()
    # env.close()
