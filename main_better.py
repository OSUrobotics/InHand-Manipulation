#!/usr/bin/env python3
import setup
import ObjectsInScene
import time
import pybullet as p

if __name__ == "__main__":
    path_to_human_data = 'Human Study Data/anjali_data_better/filt_josh_2v2_c_none_1.csv'
    human_data = setup.read_file(path_to_human_data)
    camera_view = "TOP"
    open_fingers_pose = (1.5708, 0.0, -1.5708, 0.0)
    start_grasp_pos = (0.733038, -1.62316, -0.733038, 1.62316) #From human data, measured by eye
    joint_names = ['Base', 'L_Prox', 'L_Dist', 'R_Prox', 'R_Dist']
    path_to_gripper_sdf = 'ExampleSimWorld-Josh/new/testing2_try.sdf'
    (physicsClient, planeID, num_objects, gripperID, objectIDs) = setup.init_sim(path_to_gripper_sdf)
    p.setRealTimeSimulation(True)
    objectID = objectIDs[0]
    print(human_data.shape)
    print("ALL IDS:{}".format((physicsClient, planeID, num_objects, gripperID, objectID)))
    setup.set_camera_view(camera_view)
    gripper = ObjectsInScene.Manipulator(gripperID, open_fingers_pose, start_grasp_pos, joint_names)
    print("Position is: {}, Orientation is: {}".format(gripper.pos, gripper.orn))
    cube = ObjectsInScene.SceneObject(objectID)
    print("Position is: {}, Orientation is: {}".format(cube.pos, cube.orn))
    print("JOINTINFO of Manip: {}".format(gripper.get_joints_info()))

    done_open = gripper.move_fingers_to_pose(gripper.open_fingers_pose)
    print("Complete Open? {}".format(done_open))

    done_grasp = gripper.move_fingers_to_pose(gripper.start_grasp_pose)
    print("Complete Grasp Object? {}".format(done_grasp))

    done_mov_obj = gripper.manipulate_obj(cube, human_data)


