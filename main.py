#!/usr/bin/env python3
import setup
import ObjectsInScene
import pybullet as p
import time
import Manipulator
import argparse
import matplotlib.pyplot as plt


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


def plot_human_data(data):
    x_data = []
    y_data = []
    scale = 0.1
    for line in data:
        x_data.append(scale*line[0])
        y_data.append(scale*line[1])
    # print("DATA 0: {},\n DATA 1: {},\n DATA: {}".format(data[:][0], data[1], data))
    plt.plot(x_data, y_data)
    plt.show()


if __name__ == "__main__":
    """
    Parse Arguments and start sim
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', default=None)
    args = parser.parse_args()
    print("ARGUMENTS PASSED BEFORE: {}".format(args))
    import_arguments(args, parser)
    print("ARGUMENTS PASSED: {}".format(args))

   # Initial arguments and setup
    human_data = setup.read_file(args.path_to_human_data)
    if "Yes" in args.plot_human_data:
        print("PLOT? {}".format(args.plot_human_data))
        plot_human_data(human_data)
    (physicsClient, planeID, num_objects, gripperID, objectIDs) = setup.init_sim(args.path_to_gripper_sdf)
    objectID = objectIDs[0]
    setup.set_camera_view(args.camera_view)
    gripper = Manipulator.Manipulator(gripperID, args.open_fingers_pose, args.start_grasp_pose)
    cube = ObjectsInScene.SceneObject(objectID)

    # Moving code
    done_open, _ = gripper.move_fingers_to_pose(gripper.open_fingers_pose, abs_tol=0.1)
    print("Complete Open? {}".format(done_open))

    done_grasp, contact_points = gripper.move_fingers_to_pose(gripper.start_grasp_pose, cube, abs_tol=0.05)
    print("Complete Grasp Object? {}, Contact  points: {}".format(done_grasp, contact_points))

    done_mov_obj = gripper.manipulate_object(cube, human_data, contact_check=True)
    time.sleep(2)
