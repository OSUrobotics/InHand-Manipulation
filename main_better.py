#!/usr/bin/env python3
import setup
import ObjectsInScene
import pybullet as p
import time

if __name__ == "__main__":
    # Initial arguments and setup
    path_to_human_data = 'Human Study Data/anjali_data_better/filt_josh_2v2_c_none_1.csv'
    human_data = setup.read_file(path_to_human_data)
    camera_view = "TOP"
    open_fingers_pose = (1.5708, 0.0, -1.5708, 0.0)
    start_grasp_pos = (0.7, -1.62316, -0.7, 1.62316)
    joint_names = ['Base', 'L_Prox', 'L_Dist', 'R_Prox', 'R_Dist']
    path_to_gripper_sdf = 'ExampleSimWorld-Josh/new/testing2_try.sdf'
    (physicsClient, planeID, num_objects, gripperID, objectIDs) = setup.init_sim(path_to_gripper_sdf)
    p.setRealTimeSimulation(False)
    objectID = objectIDs[0]
    print(human_data.shape)
    setup.set_camera_view(camera_view)
    gripper = ObjectsInScene.Manipulator(gripperID, open_fingers_pose, start_grasp_pos, joint_names)
    cube = ObjectsInScene.SceneObject(objectID)

    #Moving code
    done_open, _ = gripper.move_fingers_to_pose(gripper.open_fingers_pose)
    print("Complete Open? {}".format(done_open))

    done_grasp, contact_points = gripper.move_fingers_to_pose(gripper.start_grasp_pose)#, objectID)
    print("Complete Grasp Object? {}, Contact  points: {}".format(done_grasp, contact_points))

    done_mov_obj = gripper.manipulate_object(cube, human_data)
    time.sleep(2)

