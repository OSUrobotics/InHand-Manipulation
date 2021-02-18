#!/usr/bin/env python3
import setup
import ObjectsInScene
import pybullet as p
import time
import Manipulator
import plot
import argparse
# import sys


def load_from_file(filename, parser, namespace):
    parser.add_argument("--path_to_human_data", default='Human Study Data/anjali_data_better/filt_josh_2v2_c_none_1.csv')
    parser.add_argument("--plot_human_data", default=False)
    parser.add_argument("--camera_view", default="TOP")
    parser.add_argument("--path_to_gripper_sdf", default='ExampleSimWorld-Josh/new/testing2_try.sdf')
    parser.add_argument("--open_fingers_pose", default=(0.17, 0.0, -0.17, 0.0))
    parser.add_argument("--start_grasp_pose", default=(0.7, -1.62316, -0.7, 1.62316))

    with open(filename, 'r') as f:
        if f.readline() == "PASS ARGUMENTS HERE:":
            pass
        else:
            parser.parse_args(f.readline().strip().split('='), namespace)


def import_arguments(args, parser):
    if args.file is not None:
        with open('setup.txt', 'w') as f:
            f.write(args.file)

    with open('setup.txt', 'r') as f:
        filename = f.read()
    load_from_file(filename, parser, args)


if __name__ == "__main__":
    """
    Parse Arguments and start sim
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', default=None)
    args = parser.parse_args()
    import_arguments(args, parser)

   # Initial arguments and setup
    human_data = setup.read_file(args.path_to_human_data)
    if args.plot_human_data:
        plot.plot_human_data(human_data)
    joint_names = ['Base', 'L_Prox', 'L_Dist', 'R_Prox', 'R_Dist']
    (physicsClient, planeID, num_objects, gripperID, objectIDs) = setup.init_sim(args.path_to_gripper_sdf)
    objectID = objectIDs[0]
    setup.set_camera_view(args.camera_view)
    gripper = Manipulator.Manipulator(gripperID, args.open_fingers_pose, args.start_grasp_pose, joint_names)
    cube = ObjectsInScene.SceneObject(objectID)

    # Moving code
    done_open, _ = gripper.move_fingers_to_pose(gripper.open_fingers_pose, abs_tol=0.1)
    print("Complete Open? {}".format(done_open))

    done_grasp, contact_points = gripper.move_fingers_to_pose(gripper.start_grasp_pose, cube, abs_tol=0.05)
    print("Complete Grasp Object? {}, Contact  points: {}".format(done_grasp, contact_points))

    done_mov_obj = gripper.manipulate_object(cube, human_data, contact_check=True)
    time.sleep(2)
