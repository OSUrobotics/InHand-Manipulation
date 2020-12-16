import pybullet as p
import time
import pybullet_data

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

######### SIDE VIEW ###########
# p.resetDebugVisualizerCamera(cameraDistance=.2, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[.1, 0, .1])
######### TOP VIEW ###########
p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=90, cameraPitch=-89.9, cameraTargetPosition=[0.1, 0, 0.5])

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
endeffectorIndex1=2
endeffectorIndex2=4 
a = [1,2,3,4]
v = [50.0,10,-10,-20]
f = [500,500,500,500]
t_pos =  [.1, 0, .1]
t_pos1 =  list(cubePos)
t_pos1[2] = 0.15
print ("POSITION IS:",t_pos1)

sphereID1 = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[1,0,0,1])
p.createMultiBody(baseVisualShapeIndex=sphereID1,basePosition=t_pos1,baseOrientation=cubeStartOrientation)


### DATA USING INV KINEMATICS ###
# jointPoses1 = (-0.6924936035557392, 6.018687007767666e-06, 0.6924921080507533, -6.0186557027200035e-06)

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
	if (done_start_cmd == 0):
		### STARTING POSITION ###
		jointPoses1 = (1.5708, 0.0, -1.5708, 0.0)
		p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.POSITION_CONTROL, targetPositions=jointPoses1, targetVelocities=[0.001,0.001,0.001,0.001])
		done_start_cmd = 1

	if (curr_js_1 > 1.49 and curr_js_2 < -1.49 and done_start_pos == 0 and done_start_cmd == 1):
		done_start_pos = 1
		### DATA FROM HUMAN STUDIES ###
		jointPoses1 = (0.733038, -1.62316, -0.733038, 1.62316)
		p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.POSITION_CONTROL, targetPositions=jointPoses1, targetVelocities=[0.001,0.001,0.001,0.001])
	
	query = p.getJointState(boxId, a[0])
	curr_js_1 = query[0]
	query = p.getJointState(boxId, a[2])
	curr_js_2 = query[0]
	print ("CURRENT JOINT STATE:",curr_js_1,"\n\n", curr_js_2)
	dist_points  = p.getClosestPoints(boxId, cubeId, 0.1, linkIndexA=2)
	dist = dist_points[0][8]
	print ("DISTANCE:",i, dist)

p.disconnect()

#########EXTRA STUFF###########

	# t_pos1 = [t_pos[0], t_pos[1]+t1, t_pos[2]-t]
	# t_pos2 = [t_pos[0], t_pos[1]-t2, t_pos[2]]
	# p.removeBody(sphereID)
	# p.createMultiBody(baseVisualShapeIndex=sphereID1,basePosition=t_pos1,baseOrientation=cubeStartOrientation)
	# p.createMultiBody(baseVisualShapeIndex=sphereID2,basePosition=t_pos2,baseOrientation=cubeStartOrientation)

	# jointPoses1 = p.calculateInverseKinematics2(bodyUniqueId=boxId,endEffectorLinkIndices=[endeffectorIndex1, endeffectorIndex2],targetPositions=[t_pos1, t_pos2])
	# p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.POSITION_CONTROL, targetPositions=jointPoses1)
	# jointPoses1 = p.calculateInverseKinematics(bodyUniqueId=boxId,endEffectorLinkIndex=endeffectorIndex2,targetPosition=t_pos2)	

	# if (i==100):
	# 	p.removeBody(boxId)
	# 	sphereID = p.createVisualShape(shapeType=p.GEOM_SPHERE,radius=0.05,rgbaColor=[1, 0, 0, 1])
	# 	p.createMultiBody(baseVisualShapeIndex=sphereID,basePosition=cubePos,baseOrientation=cubeOrn)
	# 	print ("Position=", cubePos)
	# 	print ("Orientation=", cubeOrn)



	# curr_js  = p.getJointState(boxId, endeffectorIndex1)

	# if (dist<0.000017):
	# 	p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.VELOCITY_CONTROL, targetVelocities=[0,0,0,0])#forces=[max_force, max_force, max_force, max_force])
	# 	# p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.TORQUE_CONTROL,forces=[max_force, max_force, max_force, max_force])
