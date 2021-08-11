#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import pandas as pd
import sys
import numpy as np

def read_file(filename):
    """
    Read in csv file  containing  information  of human studies as a panda dataframe.
    Convert it  to numpy arrays
    Format: Start pos of hand is origin
    x,y,rmag,f_x,f_y,f_rmag
    Note: c dir is +x; a dir is +y [from humanstudy data]
    :param filename: Name of file containing human data
    :return: numpy  array containing the information from the file
    """
    df = pd.read_csv(filename)
    df.drop(df.columns[df.columns.str.contains('unnamed', case=False)], axis=1, inplace=True)
    # print("Head of file: \n", df.head())
    data = df.to_numpy()
    return data


def load_object_sdf(filename):
    """
    Loads an object(a) into the scene
    :param filename: Name of sdf file
    :return: num_objs: Number of objects in the scene, including gripper (based  off of  given sdf  file)
    :return: gripper_id: Reference variable to gripper in scene(Should be the first value returned by loadSDF)
    :return: obj_id: Reference variable to object(s) in scene
    """
    all_objects = p.loadSDF(filename)
    p.resetBasePositionAndOrientation(all_objects[0], p.getBasePositionAndOrientation(all_objects[0])[0],
                                      p.getQuaternionFromEuler([0, np.pi/2, np.pi/2]))
    num_objs = len(all_objects)
    p.resetBasePositionAndOrientation(all_objects[1], [0.00, 0.12, 0], p.getBasePositionAndOrientation(all_objects[1])[1])
    print("@@@@!!!####HAND: {}\nCUBE: {}".format(p.getBasePositionAndOrientation(all_objects[0]),
                                                 p.getBasePositionAndOrientation(all_objects[1])))
    gripper_id = all_objects[0]
    if num_objs > 1:
        objects = []
        for i in range(1, num_objs):
            objects.append(all_objects[i])
    else:
        objects = None
    p.changeVisualShape(gripper_id, 0, rgbaColor=[1, 0, 0, 1])
    p.changeVisualShape(gripper_id, 1, rgbaColor=[0, 1, 0, 1])
    p.changeVisualShape(gripper_id, 2, rgbaColor=[1, 0, 0, 1])
    p.changeVisualShape(gripper_id, 3, rgbaColor=[0, 1, 0, 1])
    p.changeVisualShape(gripper_id, 4, rgbaColor=[1, 0, 0, 1])
    p.changeVisualShape(objects[0], -1, rgbaColor=[0, 0, 1, 1])
    return num_objs, gripper_id, objects


def load_urdfs(filenames_list):
    """
    Loads all objects into the scene
    :param filename: Name of urdf files as a list. First file is always th ehand file. Rest are object files.
    :return: num_objs: Number of objects in the scene, including gripper
    :return: gripper_id: Reference variable to gripper in scene(Should be the first value returned by loadSDF)
    :return: obj_id: Reference variable to object(s) in scene
    """
    gripper_id = p.loadURDF(filenames_list[0], useFixedBase=True, basePosition=[0.0, 0.0, 0.04])
    p.changeVisualShape(gripper_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])
    p.changeVisualShape(gripper_id, 0, rgbaColor=[1, 0.5, 0, 1])
    p.changeVisualShape(gripper_id, 1, rgbaColor=[0.3, 0.3, 0.3, 1])
    p.changeVisualShape(gripper_id, 2, rgbaColor=[1, 0.5, 0, 1])
    p.changeVisualShape(gripper_id, 3, rgbaColor=[0.3, 0.3, 0.3, 1])


    num_objs = len(filenames_list)
    if num_objs > 1:
        objects = []
        for i in range(1, num_objs):
            objects.append(p.loadSDF(filenames_list[i])[0])
            print("OBJEVTS: {}".format(objects[i-1]))
            p.resetBasePositionAndOrientation(objects[i-1], [0.00, 0.17, 0.0], [0.706825181105366, 0.0, 0.0, 0.7073882691671998])
            # objects.append(p.loadURDF(filenames_list[i], basePosition=[0.01, 0.16, 0], useFixedBase=False))
    else:
        objects = None
    p.changeVisualShape(objects[0], -1, rgbaColor=[0, 0.75, 0.5, 1])
    return num_objs, gripper_id, objects


def init_sim(filename=None):
    """
    Initializing simulator with hand, ground plane, object
    :param filename: Name of sdf file containing objects  to load in the scene
    :return: all_ids: Reference variable to the scene, components of the scene and number of objects in the scene
    (ORDER: physics_client_id, plane_id, No. Of Objects(including gripper), gripper_id, [object_id])
    """
    physics_client_id = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.setGravity(0, 0, -10)
    plane_id = p.loadURDF("plane.urdf")
    if filename is not None:
        # no_of_objects, gripper_id, object_id = load_object_sdf(filename)
        no_of_objects, gripper_id, object_id = load_urdfs(filenames_list=filename)
    else:
        no_of_objects = 0
        gripper_id = None
        object_id = None
    all_ids = (physics_client_id, plane_id, no_of_objects, gripper_id, object_id)
    return all_ids


def set_camera_view(view="TOP"):
    """
    Set the  viewing angle of the scene
    :param view: takes two keywords ("TOP" or "SIDE")  indicative of scene view
    """
    if view == "TOP":
        p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9,
                                     cameraTargetPosition=[0, 0.1, 0.5]) #cameraTargetPosition=[0.1, 0, 0.5])
    elif view == "SIDE":
        p.resetDebugVisualizerCamera(cameraDistance=.2, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[.1, 0, .1])
    else:
        print("Not a valid input")
        raise ValueError


def quit_sim():
    """
    disconnect the sim
    TODO: Is this the best way for a clean exit?
    """
    p.disconnect()
    sys.exit()
