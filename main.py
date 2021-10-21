#!/usr/bin/env python3
import setup
import ObjectsInScene
import pybullet as p
import time
import Manipulator
import argparse
import Markers


def get_start_pose_from_dir(direction):
    if direction == 'a':
        l_prox, l_dist = 1.04, -1.57
        r_prox, r_dist = -l_prox, -l_dist
        start_contact_pose = [l_prox, l_dist, r_prox, r_dist]

    elif direction == 'b':
        l_prox, l_dist = 0.92, -1.54
        r_prox, r_dist = -l_prox, -l_dist
        start_contact_pose = [l_prox, l_dist, r_prox, r_dist]

    elif direction == 'c':
        # Josh n c 4
        l_prox, l_dist = 0.99, -1.52 # 0.25, -0.5
        r_prox, r_dist = -0.78, 1.403 # -0.909, 1.52
        start_contact_pose = [l_prox, l_dist, r_prox, r_dist]

    elif direction == 'd':
        l_prox, l_dist = 1.04, -1.57
        r_prox, r_dist = -l_prox, -l_dist
        start_contact_pose = [l_prox, l_dist, r_prox, r_dist]

    elif direction == 'e':
        l_prox, l_dist = 1.04, -1.57
        r_prox, r_dist = -l_prox, -l_dist
        start_contact_pose = [l_prox, l_dist, r_prox, r_dist]

    elif direction == 'f':
        l_prox, l_dist = 1.04, -1.57
        r_prox, r_dist = -l_prox, -l_dist
        start_contact_pose = [l_prox, l_dist, r_prox, r_dist]

    elif direction == 'g':
        l_prox, l_dist = 1.04, -1.57
        r_prox, r_dist = -l_prox, -l_dist
        start_contact_pose = [l_prox, l_dist, r_prox, r_dist]

    elif direction == 'h':
        l_prox, l_dist = 1.04, -1.57
        r_prox, r_dist = -l_prox, -l_dist
        start_contact_pose = [l_prox, l_dist, r_prox, r_dist]

    else:
        print("Invalid Direction Given!!!")
        raise KeyError
    return start_contact_pose


def load_from_file(filename, parser, namespace):
    parser.add_argument("--path_to_human_data",
                        default='Human Study Data/anjali_data_better/filt_josh_2v2_c_none_1.csv')
    parser.add_argument("--camera_view", default="TOP")
    parser.add_argument("--path_to_gripper_sdf", default='ExampleSimWorld-Josh/new/testing2_try.sdf')
    parser.add_argument("--path_to_object_sdf", default='ExampleSimWorld-Josh/2v2_hands/999/object_only.sdf')
    parser.add_argument("--open_fingers_pose", action='append', type=float)
    parser.add_argument("--start_grasp_pose", action='append', type=float)
    parser.add_argument("--path_to_gripper_urdf", default='ExampleSimWorld-Josh/2v2_test_hand_anjali/2v2_test_hand.urdf'
                        )
    parser.add_argument("--path_to_object_urdf", default='ExampleSimWorld-Josh/2v2_test_hand_object_anjali/2v2_test_'
                                                         'hand_cuboid_small.urdf')

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
    # """
    # Start GUI Interface
    # """
    # # GUI Stuff
    # app = QApplication([])
    # interface = gui.GUI()
    # interface.show()
    # app.exec_()

    """
    Parse Arguments and start sim
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', default=None)
    args = parser.parse_args()
    print("ARGUMENTS PASSED BEFORE: {}".format(args))
    import_arguments(args, parser)
    print("ARGUMENTS PASSED: {}".format(args))

    # WITHOUT GYM:
    # Initial arguments and setup
    human_data = setup.read_file(args.path_to_human_data)

    # # Only plotting pre-existing files
    controller_data = setup.read_file('controller.csv')

    # (physicsClient, planeID, num_objects, gripperID, objectIDs) = setup.init_sim(args.path_to_gripper_sdf)
    (physicsClient, planeID, num_objects, gripperID, objectIDs) = setup.init_sim([args.path_to_gripper_urdf,
                                                                                  args.path_to_object_urdf])
    objectID = objectIDs[0]
    setup.set_camera_view(args.camera_view)

    if 'exp' in args.path_to_human_data:
        split_at = 2
        type = 'expected'
    else:
        split_at = 4
        type = 'human'
    parse_file_name = args.path_to_human_data.split('/')[split_at].split('.')[0]
    man_dir = parse_file_name.split('_')[2]
    start_pose = get_start_pose_from_dir(man_dir)
    gripper = Manipulator.Manipulator(gripperID, args.open_fingers_pose, start_pose)
    gripper.limit_data = 1
    gripper.ep_step = 1
    cube = ObjectsInScene.SceneObject(objectID)

    gripper.hand_type = 'new_hand_{}'.format(type)

    # Moving code
    gripper.phase = 'Open'
    gripper.human_data_file_name = parse_file_name
    print("##!!!!!!@@@@@@@@", gripper.human_data_file_name, gripper.hand_type)
    gripper.human_data = human_data
    done_open, _ = gripper.move_fingers_to_pose(gripper.open_fingers_pose, abs_tol=0.1)
    print("Complete Open? {}".format(done_open))
    # cube.start_pos =
    p.resetBasePositionAndOrientation(objectID, cube.start_pos, cube.start_orn)

    gripper.phase = 'Close'
    done_grasp, contact_points = gripper.move_fingers_to_pose(gripper.start_grasp_pose, cube, abs_tol=0.001)
    print("Complete Grasp Object? {}, Contact  points: {}".format(done_grasp, contact_points))

    roll_fric = 0.01
    p.changeDynamics(objectID, -1, mass=0.1, rollingFriction=roll_fric)
    p.changeDynamics(gripperID, 1, rollingFriction=roll_fric)
    p.changeDynamics(gripperID, 3, rollingFriction=roll_fric)

    l_cp_info = p.getContactPoints(objectID, gripperID, linkIndexB=gripper.joint_dict[b'l_distal_pin'])
    r_cp_info = p.getContactPoints(objectID, gripperID, linkIndexB=gripper.joint_dict[b'r_distal_pin'])
    l_link_state = p.getLinkState(gripperID, gripper.joint_dict[b'l_distal_pin'])
    r_link_state = p.getLinkState(gripperID, gripper.joint_dict[b'r_distal_pin'])
    l_link_origin = p.invertTransform(l_link_state[0], l_link_state[1])
    r_link_origin = p.invertTransform(r_link_state[0], r_link_state[1])
    l_link_l_cp = p.multiplyTransforms(l_link_origin[0], l_link_origin[1], l_cp_info[0][5], cube.curr_orn)
    r_link_r_cp = p.multiplyTransforms(r_link_origin[0], r_link_origin[1], r_cp_info[0][5], cube.curr_orn)

    cube_origin = p.invertTransform(cube.curr_pos, cube.curr_orn)
    cube_l_cp = p.multiplyTransforms(cube_origin[0], cube_origin[1], l_cp_info[0][5], cube.curr_orn)
    cube_r_cp = p.multiplyTransforms(cube_origin[0], cube_origin[1], r_cp_info[0][5], cube.curr_orn)


    p.createConstraint(objectID, -1, gripperID, gripper.joint_dict[b'l_distal_pin'], p.JOINT_PRISMATIC, [0.0, 0.0, 0],
                       cube_l_cp[0], l_link_l_cp[0], parentFrameOrientation=cube_l_cp[1],
                       childFrameOrientation=l_link_l_cp[1])
    p.createConstraint(objectID, -1, gripperID, gripper.joint_dict[b'r_distal_pin'], p.JOINT_PRISMATIC, [0.0, 0.0, 0],
                       cube_r_cp[0], r_link_r_cp[0], parentFrameOrientation=cube_r_cp[1],
                       childFrameOrientation=r_link_r_cp[1])

    gripper.phase = 'Move'
    done_mov_obj = gripper.manipulate_object(cube, human_data, contact_check=True)
    time.sleep(2)

# #WITH GYM:
#
#     """
#     For importing gym env
#     """
#     env = gym.make("ihm-v0", kwargs={'args': args})
#     env.reset()
#     # done_mov_obj = env.gripper.manipulate_object(env.cube, env.human_data, contact_check=True)
#     i = 0
#     # action = np.array([0.17, 0.0, -0.17, 0.0])
#     while i < 3000:
#         # print(i)
#         obs, reward, done, info = env.step(action=env.action_space.sample())
#         i += 1
#         print("EPISODE COUNT: {}, DONE BIT: {}, REWARD: {}, ITER: {}".format(env.episode_count, done, reward, i))
#         if done:
#             obs = env.reset()
#     env.close()
