#!/usr/bin/env python

import pybullet as p
from time import sleep
from mamad_util import JointInfo
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
#print ("current_dir=" + currentdir)
os.sys.path.insert(0,currentdir)
import pybullet_data
from pkg_resources import parse_version

import sys

def setup_GUI_slidbars(Robot,RobotId,cubeId):
	ids =[]

	cube_pos,cube_orn = p.getBasePositionAndOrientation(cubeId)
	# kuka_ee_angle
	
	ee_orn = [0,0,0]
	ee_pos = [cube_pos[0],cube_pos[1],cube_pos[2]+0.3]
	slider_names = ["x","y","z","r","p","y"]
	sliders = {
		"x":{"initial":ee_pos[0],"ll":ee_pos[0]-2,"ul":ee_pos[0]+2},
		"y":{"initial":ee_pos[1],"ll":ee_pos[1]-2,"ul":ee_pos[1]+2},
		"z":{"initial":ee_pos[2],"ll":ee_pos[2]-2,"ul":ee_pos[2]+2},
		"r":{"initial":ee_orn[0],"ll":-3.14,"ul":3.14},
		"p":{"initial":ee_orn[1],"ll":-3.14,"ul":3.14},
		"y":{"initial":ee_orn[2],"ll":-3.14,"ul":3.14}

	}
    #setting up gui slider for each active joint
	for key in slider_names:
		sliderID = p.addUserDebugParameter(key,sliders[key]["ll"],sliders[key]["ul"],sliders[key]["initial"])
		ids.append(sliderID)
	return ids


def read_GUI_slidbars(sliders_id):

	slider_value = []
	for id in sliders_id:
		slider_value.append( p.readUserDebugParameter(id))
    
	ee_pos = slider_value[:3]
	ee_orn = slider_value[3:]
	ee_orn = p.getQuaternionFromEuler(ee_orn)
	return ee_pos,ee_orn


def check_collision(RobotId,cubeId):
	contact = p.getClosestPoints(RobotId,cubeId,0.00001)
	print("contact::: ",contact)

p.connect(p.GUI)
robot = p.loadSDF("./model.sdf")
robotID = robot[0]
print("robotID:::",robotID)
p.resetBasePositionAndOrientation(robotID,[0.00000,0.200000,0.65000],[0.000000,0.000000,0.000000,1.000000])
table = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"table/table.urdf"), 0.5000000,0.60000,0.0000,0.000000,0.000000,0.0,1.0)
texUid = p.loadTexture("./cube/aaa.png")
cube_objects = p.loadSDF("./cube/model.sdf")
cubeId = cube_objects[0]
p.changeVisualShape(cube_objects[0], -1,rgbaColor =[1,1,1,1])
p.changeVisualShape(cube_objects[0], -1, textureUniqueId = texUid)
p.resetBasePositionAndOrientation(cube_objects[0],(0.5000000,0.60000,0.6700),(0.717,0.0,0.0,0.717))



p.loadURDF(os.path.join(pybullet_data.getDataPath(),"plane.urdf"),[0,0,0])

jointInfo = JointInfo()
jointInfo.get_infoForAll_joints(robot)
active_joints_info  = jointInfo.getActiveJointsInfo()
num_active_joints = jointInfo.getNumberOfActiveJoints()

#*******************Getting Box_ee info*************************
link_name ="Box_ee"
link_name_encoded = link_name.encode(encoding='UTF-8',errors='strict')
Box_ee_jointInfo = jointInfo.searchBy(key="linkName",value =link_name_encoded )[0]
Box_ee_Index  = Box_ee_jointInfo["jointIndex"]
print("Box_ee_Index:: ",Box_ee_Index)
print("Box_ee_jointInfo::  ",Box_ee_jointInfo)
# sys.exit()
#***************************************************************
"""

num_joints = p.getNumJoints(robotID)
#print("`num of joints:::",num_joints)

for i in range(num_joints-1):
	j_info = p.getJointInfo(robotID,i)
	#print("joint_info::",j_info)
"""
p.setRealTimeSimulation(0)


link_name ="lbr_iiwa_link_7"
link_name_encoded = link_name.encode(encoding='UTF-8',errors='strict')
kuka_ee_link_jointInfo = jointInfo.searchBy(key="linkName",value =link_name_encoded )[0]
#print(kuka_ee_link_jointInfo)
kuka_ee_link_Index = kuka_ee_link_jointInfo["jointIndex"]



sliders_id = setup_GUI_slidbars(robot,robotID,cubeId)
sleep(1.)
while(1):
	ee_pos,ee_orn = read_GUI_slidbars(sliders_id)
	jointPoses  = p.calculateInverseKinematics(robotID,kuka_ee_link_Index,ee_pos,ee_orn)	

	for i in range(num_active_joints):
		jointIndex = active_joints_info[i]["jointIndex"]
		jointll = active_joints_info[i]["jointLowerLimit"]
		jointul = active_joints_info[i]["jointUpperLimit"]
		motor_command = jointPoses[i]
		p.setJointMotorControl2(robotID,jointIndex,p.POSITION_CONTROL,motor_command, targetVelocity=0,force=200.0, maxVelocity=0.35,positionGain=0.3,velocityGain=1)
	check_collision(robotID,cubeId)
	p.stepSimulation()
	
	


