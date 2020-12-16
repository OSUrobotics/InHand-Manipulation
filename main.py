import pybullet as p
import time
import pybullet_data
import pandas as pd 
import numpy as np

###Read in Human data as panda dataframe
path_to_human_data  = '../Human Study Data/anjali_data_better/filt_josh_2v2_c_none_3.csv'
df = pd.read_csv(path_to_human_data)
df.drop(df.columns[df.columns.str.contains('unnamed',case = False)],axis = 1, inplace = True)
print ("Head of file: \n", df.head())

### Convert panda dataframe to numpy array
dist_val = df.to_numpy()
print ("Type: \n",type(dist_val))
print ("Shape: \n",dist_val.shape)

###Initializing simulator with hand, ground plane, object
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally 
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
gripper  = p.loadSDF("../ExampleSimWorld-Josh/new/testing2_try.sdf")
print  ('DESCRIPTION \n', gripper, '\n END')
boxId = gripper [0]
cubeId = gripper[1]
cubePos, cubeOrn  =  p.getBasePositionAndOrientation(cubeId)
print ("WHATS THIS", type(cubePos))

###Setting up the camera view
######### SIDE VIEW ###########
# p.resetDebugVisualizerCamera(cameraDistance=.2, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[.1, 0, .1])
######### TOP VIEW ###########
p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=90, cameraPitch=-89.9, cameraTargetPosition=[0.1, 0, 0.5])

###Hand description
numjoints = p.getNumJoints(boxId)
print ("No. of Joints:", numjoints)
jointinfo = p.getJointInfo(boxId,0)
print ("Joint Info:", jointinfo) 
jointinfo = p.getJointInfo(boxId,1)
print ("Joint Info:", jointinfo) 
jointinfo = p.getJointInfo(boxId,2)
print ("Joint Info:", jointinfo) 
jointinfo = p.getJointInfo(boxId,3)
print ("Joint Info:", jointinfo) 
jointinfo = p.getJointInfo(boxId,4)
print ("Joint Info:", jointinfo) 

###Easy referecncing of end effector link numbers
endeffectorIndex1=2
endeffectorIndex2=4 

###Joint link  indices array for easy referencing
a = [1,2,3,4]

###To  set  velocity
v = [50.0,10,-10,-20]

###To set force
f = [500,500,500,500]

###Setting position  of the sphere to track changes
t_pos =  list(cubePos)
t_pos[2] = 0.15
print ("POSITION IS:",t_pos)

###Creating the  sphere
sphereID1 = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[1,0,0,1])
p.createMultiBody(baseVisualShapeIndex=sphereID1,basePosition=t_pos,baseOrientation=cubeStartOrientation)

### DATA USING INV KINEMATICS ###
#inv_kin_data = (-0.6924936035557392, 6.018687007767666e-06, 0.6924921080507533, -6.0186557027200035e-06)
### DATA FROM HUMAN STUDIES ###
human_data = (0.733038, -1.62316, -0.733038, 1.62316)

max_force = 0.0001 
done_start_pos = 0
done_start_cmd = 0

for i in range (150):
	p.stepSimulation()
	time.sleep(1./240.)

	query = p.getJointState(boxId, a[0])
	curr_js_1 = query[0]
	query = p.getJointState(boxId, a[2])
	curr_js_2 = query[0]

	### STARTING POSITION ###
	if (done_start_cmd == 0):

		###Set start position of hand parallel to  object.
		jointPoses1 = (1.5708, 0.0, -1.5708, 0.0)
		p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.POSITION_CONTROL, targetPositions=jointPoses1, targetVelocities=[0.001,0.001,0.001,0.001])
		done_start_cmd = 1

	### PRE-MANIPULATION POSE ###
	if (curr_js_1 > 1.49 and curr_js_2 < -1.49 and done_start_pos == 0 and done_start_cmd == 1):
		done_start_pos = 1
		
		### Reach the  grasping position (pre-manipulation pose)
		jointPoses1 = human_data
		p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.POSITION_CONTROL, targetPositions=jointPoses1, targetVelocities=[0.001,0.001,0.001,0.001])
	
	query = p.getJointState(boxId, a[0])
	curr_js_1 = query[0]
	query = p.getJointState(boxId, a[2])
	curr_js_2 = query[0]
	print ("CURRENT JOINT STATE:",curr_js_1,"\n\n", curr_js_2)

	###Checking closest  distance between hand and obj
	dist_points  = p.getClosestPoints(boxId, cubeId, 0.1, linkIndexA=2)
	dist = dist_points[0][8]
	print ("DISTANCE:",i, dist)

p.disconnect()














#########EXTRA STUFF###########
###Removing a body
	# p.removeBody(sphereID)
###Performing inverse kinematics
	# jointPoses1 = p.calculateInverseKinematics2(bodyUniqueId=boxId,endEffectorLinkIndices=[endeffectorIndex1, endeffectorIndex2],targetPositions=[t_pos1, t_pos2])
	# p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.POSITION_CONTROL, targetPositions=jointPoses1)
	# jointPoses1 = p.calculateInverseKinematics(bodyUniqueId=boxId,endEffectorLinkIndex=endeffectorIndex2,targetPosition=t_pos2)	
