import pybullet as p
import time
import pybullet_data
import pandas as pd
import numpy as np
import time

# Read in Human data as panda data frame ###
path_to_human_data = 'Human Study Data/anjali_data_better/filt_josh_2v2_c_none_1.csv'
df = pd.read_csv(path_to_human_data)
df.drop(df.columns[df.columns.str.contains('unnamed', case=False)], axis=1, inplace=True)
print("Head of file: \n", df.head())

# Convert panda data frame to numpy array ###
dist_val = df.to_numpy()
print("Type: \n", type(dist_val))
print("Shape: \n", dist_val.shape)
print("Length: \n", len(dist_val))

# Initializing simulator with hand, ground plane, object ###
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version 
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally 
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
gripper = p.loadSDF("ExampleSimWorld-Josh/new/testing2_try.sdf")
print('DESCRIPTION \n', gripper, '\n END')
boxId = gripper[0]
cubeId = gripper[1]
cubePos, cubeOrn = p.getBasePositionAndOrientation(cubeId)
print("WHATS THIS", type(cubePos))

# Setting up the camera view ###
# SIDE VIEW ###########
# p.resetDebugVisualizerCamera(cameraDistance=.2, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[.1, 0, .1])
# TOP VIEW ###########
p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=90, cameraPitch=-89.9, cameraTargetPosition=[0.1, 0, 0.5])

# Hand description ###
num_joints = p.getNumJoints(boxId)
print("No. of Joints:", num_joints)
joint_info = p.getJointInfo(boxId, 0)
print("Joint Info:", joint_info)
joint_info = p.getJointInfo(boxId, 1)
print("Joint Info:", joint_info)
joint_info = p.getJointInfo(boxId, 2)
print("Joint Info:", joint_info)
joint_info = p.getJointInfo(boxId, 3)
print("Joint Info:", joint_info)
joint_info = p.getJointInfo(boxId, 4)
print("Joint Info:", joint_info)

# Easy referencing of end effector link numbers ###
endEffectorIndex1 = 2
endEffectorIndex2 = 4

# Joint link  indices array for easy referencing ###
a = [1, 2, 3, 4]

# To  set  velocity ###
v = [50.0, 10, -10, -20]

# To set force ###
f = [500, 500, 500, 500]

# Setting position  of the sphere to track changes ###
t_pos1 = list(cubePos)
t_pos = t_pos1
t_pos[2] = 0.15
print("POSITION IS:", t_pos)

# Creating the  sphere ###
# sphereID = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[1, 0, 0, 1])
# p.createMultiBody(baseVisualShapeIndex=sphereID, basePosition=t_pos, baseOrientation=cubeStartOrientation)
# sphereID1 = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[1, 0, 0, 1])
# sphereID2 = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[1, 0, 0, 1])
# p.createMultiBody(baseVisualShapeIndex=sphereID2, basePosition=t_pos, baseOrientation=cubeStartOrientation)

# DATA USING INV KINEMATICS ###
# inv_kin_data = (-0.6924936035557392, 6.018687007767666e-06, 0.6924921080507533, -6.0186557027200035e-06)
# DATA FROM HUMAN STUDIES ###
human_data = (0.733038, -1.62316, -0.733038, 1.62316)

vel = 0.01
force = 10
vel_array = [vel, vel, vel, vel]
force_array = [force, force, force, force]
max_force = 0.0001
done_start_pos = 0
done_preman_pos = 0
done_move_obj = 0
j = 10
sim_steps = 300
scale = 0.1
j_max = len(dist_val)
find_cube_dist = 0.042

trans_mat = np.zeros((3, 3))
print(trans_mat)
start_pos_mat = np.array([[t_pos1[0]], [t_pos1[1]], [1]])
print("START POS", start_pos_mat)
# print(trans_mat[0][0])
contact_points_r = [0.07689960296112311, -0.05757622451707545, 0.042152355722620966]
contact_points_l = [0.0769317965841001, 0.05757578969933764, 0.03833692008055983]

tx_r = start_pos_mat[0][0] #- contact_points_r[0]
ty_r = contact_points_r[1] - start_pos_mat[1][0]
rot_r = np.arctan(ty_r/tx_r)
rot_r_cos = 0#np.cos(rot_r)
rot_r_sin = 0#np.sin(rot_r)

trans_mat_cp = np.array([[rot_r_cos, -rot_r_sin, tx_r], [rot_r_sin, rot_r_cos, ty_r], [0, 0, 1]])
# trans_mat_cp = np.array([[rot_r_cos, -rot_r_sin, tx_r], [rot_r_sin, rot_r_cos, ty_r], [0, 0, 1]])
print("CONTACT TRANSFORM:", trans_mat_cp)
new_points = np.matmul(trans_mat_cp, start_pos_mat)
print("NEW POINTS:", new_points)
try_point = np.array([[0.0081],[0.0306],[1]])
try_this = np.matmul(trans_mat_cp,try_point)
print("TRY  POINT:", try_this)

sphereID2 = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[1, 0, 0, 1])
# p.createMultiBody(baseVisualShapeIndex=sphereID2, basePosition=(new_points[0][0], new_points[1][0], 0.17),
# baseOrientation=cubeStartOrientation)
# p.createMultiBody(baseVisualShapeIndex=sphereID2, basePosition=(start_pos_mat[0][0], start_pos_mat[1][0], 0.17),
# baseOrientation=cubeStartOrientation)
p.createMultiBody(baseVisualShapeIndex=sphereID2, basePosition=(0, 0, 0.17), baseOrientation=cubeStartOrientation)

# p.createMultiBody(baseVisualShapeIndex=sphereID2, basePosition=(contact_points_r[0], contact_points_r[1], 0.15),
# baseOrientation=cubeStartOrientation)

for i in range(sim_steps):
    p.stepSimulation()
    time.sleep(1. / 240.)

    query = p.getJointState(boxId, a[0])
    curr_js_1 = query[0]
    query = p.getJointState(boxId, a[2])
    curr_js_2 = query[0]

    # Checking closest  distance between hand and obj 
    dist_points = p.getClosestPoints(boxId, cubeId, 0.1, linkIndexA=2)
    # print(len(dist_points), dist_points[0][8])
    dist = dist_points[0][8]

    # STARTING POSITION ###
    if done_start_pos == 0:
        print("IM IN 1")
        # Set start position of hand parallel to  object.
        jointPoses1 = (1.5708, 0.0, -1.5708, 0.0)
        p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.POSITION_CONTROL,
                                    targetPositions=jointPoses1, targetVelocities=vel_array, forces=force_array)
        done_start_pos = 1

    # PRE-MANIPULATION POSE ###
    if curr_js_1 > 1.49 and curr_js_2 < -1.49 and done_preman_pos == 0 and done_start_pos == 1:
        print("IM IN 2")
        done_preman_pos = 1
        # Reach the  grasping position (pre-manipulation pose)
        jointPoses1 = human_data
        p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.POSITION_CONTROL,
                                    targetPositions=jointPoses1, targetVelocities=vel_array, forces=force_array)

"""
    # MOVING OBJECT IN HAND ###
    if done_preman_pos and dist < 0.003 and j < j_max:
        print("IM IN 3")
        done_move_obj = 1

        # Getting the Contact information ###
        # contact_points_info = p.getContactPoints(cubeId, boxId, linkIndexB=endEffectorIndex2)
        # try:
        #     contact_points = contact_points_info[0][5]
        #     print (contact_points)
        # except IndexError:
        #     print("Not in contact as yet")

        x_inc = dist_val[j][1] * scale
        y_inc = dist_val[j][0] * scale
        print("POSITION VALUES:", j, x_inc, "\t", y_inc, t_pos[2])
        # rot_ang = np.radians(dist_val[j][2])
        # cos_ang = np.cos(rot_ang)
        # sin_ang = np.sin(rot_ang)
        #
        # trans_mat[0][0] = cos_ang
        # trans_mat[0][1] = -sin_ang
        # trans_mat[1][0] = sin_ang
        # trans_mat[1][1] = cos_ang
        # trans_mat[0][2] = x_inc
        # trans_mat[1][2] = y_inc
        # trans_mat[2][2] = 1
        # print(trans_mat)
        #
        # new_coord_mat = np.matmul(trans_mat, start_pos_mat)
        # new_coord_mat[2][0] = t_pos[2]

        t_pos = np.array([[t_pos1[0] + x_inc], [t_pos1[1] + y_inc], [1]])
        t_pos_r = np.matmul(trans_mat_cp, t_pos)
        print("New Points Are:", t_pos, t_pos.shape)
        t_pos_l = [t_pos1[0] + x_inc, t_pos1[1] + y_inc, t_pos[2]]
        # t_pos_r = [t_pos1[0] + x_inc, t_pos1[1] + y_inc, t_pos[2]]

        # t_pos_l = [new_coord_mat[0][0], new_coord_mat[1][0] + find_cube_dist, new_coord_mat[2][0]]
        # t_pos_r = [0.07689960296112311, -0.05757622451707545, 0.042152355722620966]#[new_coord_mat[0][0], 
        new_coord_mat[1][0] - find_cube_dist, new_coord_mat[2][0]]
        # new_mat = [new_coord_mat[0][0], new_coord_mat[1][0], new_coord_mat[2][0]]
        # print("TRANSFORMED VALUES:", new_coord_mat.shape, "\n", new_coord_mat, t_pos_l)

        # p.removeBody(sphereID1)
        # sphereID1 = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[1, 0, 0, 1])
        # p.createMultiBody(baseVisualShapeIndex=sphereID1, basePosition=t_pos_l, baseOrientation=cubeStartOrientation)
        # #p.removeBody(sphereID2)
        sphereID2 = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[0, 1, 0, 1])
        p.createMultiBody(baseVisualShapeIndex=sphereID2, basePosition=[t_pos[0][0], t_pos[1][0], 0.15], 
        baseOrientation=cubeStartOrientation)

        j += 1
        # jointPoses1 = p.calculateInverseKinematics2(bodyUniqueId=boxId,
        #                                             endEffectorLinkIndices=[endEffectorIndex1, endEffectorIndex2],
        #                                             targetPositions=[t_pos_r, t_pos_l])
        # p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.POSITION_CONTROL,
        #                             targetPositions=jointPoses1)  # , targetVelocities=vel_array, forces = force_array)

    if j >= j_max and done_move_obj:
        sphereID1 = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[1, 0, 0, 1])
        p.createMultiBody(baseVisualShapeIndex=sphereID1, basePosition=t_pos_l, baseOrientation=cubeStartOrientation)
        sphereID2 = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[1, 0, 0, 1])
        p.createMultiBody(baseVisualShapeIndex=sphereID2, basePosition=t_pos_r, baseOrientation=cubeStartOrientation)
        time.sleep(2)
        break

query = p.getJointState(boxId, a[0])
curr_js_1 = query[0]
query = p.getJointState(boxId, a[2])
curr_js_2 = query[0]
print("CURRENT JOINT STATE:", curr_js_1, "\n\n", curr_js_2)

joint_info = p.getJointInfo(boxId, a[0])
print("JOINT INFO:\n", joint_info)

# Checking closest  distance between hand and obj ###
dist_points = p.getClosestPoints(boxId, cubeId, 0.1, linkIndexA=2)
dist = dist_points[0][8]
print("DISTANCE:", i, dist)

p.disconnect()

#########EXTRA STUFF###########
###Removing a body
# p.removeBody(sphereID)
###Performing inverse kinematics
# jointPoses1 = p.calculateInverseKinematics2(bodyUniqueId=boxId,endEffectorLinkIndices=[endEffectorIndex1, 
endEffectorIndex2],targetPositions=[t_pos1, t_pos2])
# p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.POSITION_CONTROL, targetPositions=jointPoses1)
# jointPoses1 = p.calculateInverseKinematics(bodyUniqueId=boxId,endEffectorLinkIndex=endEffectorIndex2,targetPosition=t_pos2)"""

