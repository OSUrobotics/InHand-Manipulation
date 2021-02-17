#!/usr/bin/env python3
import setup
import ObjectsInScene
import pybullet as p
import time
import Manipulator
import plot

if __name__ == "__main__":
    """
    TODO: Convert these arguments as input form a file/ Macros?
    """
    # Initial arguments and setup
    path_to_human_data = 'Human Study Data/anjali_data_better/filt_josh_2v2_c_none_1.csv'
    human_data = setup.read_file(path_to_human_data)
    plot.plot_human_data(human_data)
    camera_view = "TOP"
    # open_fingers_pose = (1.5708, 0.0, -1.5708, 0.0)
    open_fingers_pose = (0.17, 0.0, -0.17, 0.0)
    start_grasp_pos = (0.7, -1.62316, -0.7, 1.62316)
    joint_names = ['Base', 'L_Prox', 'L_Dist', 'R_Prox', 'R_Dist']
    path_to_gripper_sdf = 'ExampleSimWorld-Josh/new/testing2_try.sdf'
    (physicsClient, planeID, num_objects, gripperID, objectIDs) = setup.init_sim(path_to_gripper_sdf)
    p.setRealTimeSimulation(False)
    objectID = objectIDs[0]
    print(human_data.shape)
    setup.set_camera_view(camera_view)
    gripper = Manipulator.Manipulator(gripperID, open_fingers_pose, start_grasp_pos, joint_names)
    # print(gripper)
    cube = ObjectsInScene.SceneObject(objectID)

    # Moving code
    done_open, _ = gripper.move_fingers_to_pose(gripper.open_fingers_pose, abs_tol=0.1)
    print("Complete Open? {}".format(done_open))

    done_grasp, contact_points = gripper.move_fingers_to_pose(gripper.start_grasp_pose, cube, abs_tol=0.05)
    print("Complete Grasp Object? {}, Contact  points: {}".format(done_grasp, contact_points))

    done_mov_obj = gripper.manipulate_object(cube, human_data, contact_check=True)
    time.sleep(2)
