#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import pandas as pd
import sys

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
    num_objs = len(all_objects)


    gripper_id = all_objects[0]
    if num_objs > 1:
        objects = []
        for i in range(1, num_objs):
            objects.append(all_objects[i])
    else:
        objects = None
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
        no_of_objects, gripper_id, object_id = load_object_sdf(filename)
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
        p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=90, cameraPitch=-89.9,
                                     cameraTargetPosition=[0.1, 0, 0.5])
    elif view == "SIDE":
        p.resetDebugVisualizerCamera(cameraDistance=.2, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[.1, 0, .1])
    else:
        print("Not a valid input")
        raise ValueError


def quit_sim():
    """
    disconnect the sim
    TODO: Is this   the best way for a clean exit?
    """
    p.disconnect()
    sys.exit()
