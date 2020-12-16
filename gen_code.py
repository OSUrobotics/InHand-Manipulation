# What do we want to generalize?
# 1) Be able to Load Different hands:
# 		a) Different No. of Links
#		b) Different No. of fingers
# 
# 2) Be able to load Different objects
#		a) Different contact points depending on object size and shape
# 3) Change the orientation of the  hand
# 4) Change the  orientation of the object
# 5) Change  the orientation of the camera
# 6) Be able to give it points to follow 














import pybullet as p
import time
import pybullet_data
import helper_functions
#-------->
# physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version 
# p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally 
# p.setGravity(0,0,-10)
# # planeId = p.loadURDF("plane.urdf")
planeId = helper_functions.setupScene()
#------->


cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("pr2_gripper.urdf",cubeStartPos, cubeStartOrientation)
# wsg50_one_motor_gripper_new.sdf
gripper  = p.loadSDF("ExampleSimWorld-Josh/123/testing.sdf")
print  ('DESCRIPTION \n', gripper, '\n END')
boxId = gripper [0]
p.resetDebugVisualizerCamera(cameraDistance=.2, cameraYaw=90, cameraPitch=0, cameraTargetPosition=[.1, 0, .1])
# p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=90, cameraPitch=-89.9, cameraTargetPosition=[0.3, 0, 1])
numjoints = p.getNumJoints(boxId)
print ("No. of Joints:", numjoints)
jointinfo = p.getJointInfo(boxId,0)#(2,6)
print ("Joint Info:", jointinfo) 
jointinfo = p.getJointInfo(boxId,1)#(2,6)
print ("Joint Info:", jointinfo) 
jointinfo = p.getJointInfo(boxId,2)#(2,6)
print ("Joint Info:", jointinfo) 
jointinfo = p.getJointInfo(boxId,3)#(2,6)
print ("Joint Info:", jointinfo) 
jointinfo = p.getJointInfo(boxId,4)#(2,6)
print ("Joint Info:", jointinfo) 
endeffectorIndex1=2
endeffectorIndex2=4 
a = [1,2,3,4]
v = [50.0,10,-10,-20]
f = [500,500,500,500]
t_pos =  [.1, 0, .1]
t_pos1 =  [.1, 0-0.01, .1]
t_pos2 =  [.1, 0+0.01, .1]

sphereID1 = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[1,0,0,1])
p.createMultiBody(baseVisualShapeIndex=sphereID1,basePosition=t_pos1,baseOrientation=cubeStartOrientation)
sphereID2 = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[0,1,0,1])
p.createMultiBody(baseVisualShapeIndex=sphereID2,basePosition=t_pos2,baseOrientation=cubeStartOrientation)

t = 0
t1 = 0
t2 = 0
for i in range (200):
	t1 =  t1 + 0.01
	t2 =  t2 + 0.01
	t = t + 0.0001
	p.stepSimulation()
	time.sleep(1./240.)
	t_pos1 = [t_pos[0], t_pos[1]+t1, t_pos[2]-t]
	t_pos2 = [t_pos[0], t_pos[1]-t2, t_pos[2]]
	# p.removeBody(sphereID)
	p.createMultiBody(baseVisualShapeIndex=sphereID1,basePosition=t_pos1,baseOrientation=cubeStartOrientation)
	p.createMultiBody(baseVisualShapeIndex=sphereID2,basePosition=t_pos2,baseOrientation=cubeStartOrientation)

	jointPoses1 = p.calculateInverseKinematics2(bodyUniqueId=boxId,endEffectorLinkIndices=[endeffectorIndex1, endeffectorIndex2],targetPositions=[t_pos1, t_pos2])
	# jointPoses1 = p.calculateInverseKinematics(bodyUniqueId=boxId,endEffectorLinkIndex=endeffectorIndex2,targetPosition=t_pos2)
	print ("JP1:",i ,jointPoses1)
	p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=a, controlMode=p.POSITION_CONTROL, targetPositions=jointPoses1)

	# if (i==100):
	# 	p.removeBody(boxId)
	# 	sphereID = p.createVisualShape(shapeType=p.GEOM_SPHERE,radius=0.05,rgbaColor=[1, 0, 0, 1])
	# 	p.createMultiBody(baseVisualShapeIndex=sphereID,basePosition=cubePos,baseOrientation=cubeOrn)
	# 	print ("Position=", cubePos)
	# 	print ("Orientation=", cubeOrn)

p.disconnect()