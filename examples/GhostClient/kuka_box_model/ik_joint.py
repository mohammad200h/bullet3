import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print ("current_dir=" + currentdir)
os.sys.path.insert(0,currentdir)
import math
import numpy as np
import multiprocessing
import time

import random

# ros imports
import sys
import rospy
from hand_service.srv import *
from collections import OrderedDict


def roslaunch(username="mamad"):
 
  os.system("terminator -x ./ik_launcher_moveit.sh "+username)
  # os.system("terminator -x ./ik_launcher.sh")

def run(username="mamad"):
  os.system("terminator -x ./ik_launcher_hand_service.sh "+username)

class IK():
  
  def __init__(self,ee_type = "kuka",username="mamad",terminator_delay=20):
    self.ee_type = ee_type
    moveit_process = multiprocessing.Process(name="Moveit",target=roslaunch,args=(username,))
    moveit_process.daemon = True
    moveit_process.start()
    time.sleep(int(terminator_delay))
    hand_service_process = multiprocessing.Process(name="Moveit",target=run,args=(username,))
    hand_service_process.daemon = True
    hand_service_process.start()
    time.sleep(int(terminator_delay/5))
  
  def ik_gaol(self,pose):
    # The palm is in fixed position giving hand time to learn how to move the fingers 
    # print("YOOOOOOOOOOOO")
    jointCommad = OrderedDict() 
    rospy.wait_for_service('getRobotEndEffector')
    try:
        getRobotEndEffector = rospy.ServiceProxy('getRobotEndEffector', moveRobot_joint)
        res = getRobotEndEffector(self.ee_type,[0,0,0,0,0,0,0])
        print(res.RobotJoints)
        for  i,jointName in enumerate(res.RobotJoints.name):
          jointCommad[jointName] = res.RobotJoints.position[i]
        return jointCommad
    except rospy.ServiceException as e:
        print("Service failed")








# testing the class
IK = IK(ee_type = "kuka")
pose = [0,-0.3,0.765,1.57,0,0]
IK.ik_gaol(pose)