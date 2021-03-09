import gym
from gym import spaces
import pybullet_envs
import time
import pybullet as p
import pybullet_data
import os, sys
import numpy as np


class IHM_BulletEnv(gym.Env):
    def __init__(self, arm_or_end_effector="hand", frame_skip=15):
        physics_client_id = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -10)
        plane_id = p.loadURDF("plane.urdf")

        self.file_dir = os.path.dirname(os.path.realpath(__file__))
        self.arm_or_hand=arm_or_end_effector
        if arm_or_end_effector == "arm":
            self._model = p.loadSDF("ExampleSimWorld-Josh/new/testing2_try.sdf")
        elif arm_or_end_effector == "hand":
             #self._model,self.obj_size,self.filename = load_model_from_path(self.file_dir + "/kinova_description/j2s7s300_end_effector_v1_scyl.xml"),'s',"/kinova_description/j2s7s300_end_effector_v1_scyl.xml"
            self._model = p.loadSDF("ExampleSimWorld-Josh/new/testing2_try.sdf")

        else:
            print("CHOOSE EITHER HAND OR ARM")
            raise ValueError
        self.action_space = spaces.Discrete(4)
        self.observation_space = spaces.Box(np.array([0,0,0]), np.array([0,0,0]))
        self.gripper_id = self._model[0]
        self.joint_indices = [1,2,3,4]

        # self._sim = MjSim(self._model)   # The simulator. This holds all the information about object locations and orientations

    def reset(self):
        pass

    def step(self, action):
        p.setJointMotorControlArray(bodyIndex=self.gripper_id, jointIndices=self.joint_indices,
                                    controlMode=p.POSITION_CONTROL, targetPositions=action)
        return [np.array([0,0,0]), np.array([0,0,0])], 5, False, 5


if __name__ == "__main__":
    pass
