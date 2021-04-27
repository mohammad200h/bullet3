#!/usr/bin/env python
import yaml
import random
import io
import subprocess
import sys

# get joint limits for hand 
# discritisize the joint interval
# do forward kinematic for combination of joints 4 nested lopps 
# store the values in a ymal 
# make a mesh out of the yaml 
# add mesh to model


import pybullet as p

from mamad_util import JointInfo

import numpy as np 
import open3d as o3d 
# from pyntcloud import PyntCloud

finger_name = ["FF","MF","RF"]
p.connect(p.GUI)
robot = p.loadSDF("./model.sdf")

print(robot)

class WorkSpace_handAttachedToKuka_one_finger():
    def __init__(self,finger_name,robot,divition=5):
        self.finger_name = finger_name
        self.robot = robot
        self.divition = divition
        self.general_links = ["knuckle_","proximal_","middle_","distal_","fingertip_"]
        self.finger_links = [ link+finger_name for link in self.general_links]
        self.finger_links_withActiveJoints = self.finger_links[:4]
        self.jointLimits = self.get_jointLimits_forLinks(self.finger_links_withActiveJoints)
      

    def get_jointLimits_forLinks(self,finger_links):

        jointInfo = JointInfo()
        jointInfo.get_infoForAll_joints(self.robot)
        active_joints_info  = jointInfo.getActiveJointsInfo()
        num_active_joints = jointInfo.getNumberOfActiveJoints()
        jointLimits = {}

            
        for link in finger_links:
            link_encoded = link.encode(encoding='UTF-8',errors='strict')
            link_joint_info = jointInfo.searchBy(key="linkName",value =link_encoded )[0]
           
            # print(link+": \n")
            # print(link_joint_info)
            # print("\n")
            jointLimits[link] ={
                "jointIndex":link_joint_info["jointIndex"],
                "jointUpperLimit":link_joint_info["jointUpperLimit"],
                "jointLowerLimit":link_joint_info["jointLowerLimit"],
                "joint_step":(link_joint_info["jointUpperLimit"]- link_joint_info["jointLowerLimit"])/self.divition
            }
  
        return jointLimits
   
        
    def get_reachable_points(self):
        reach_able_points = []
        j_values = [self.jointLimits[l]["jointLowerLimit"] for l in self.finger_links_withActiveJoints]
        initial_j_values = j_values[:]
        j_step = [self.jointLimits[l]["joint_step"] for l in self.finger_links_withActiveJoints]

        j_values_upper = [self.jointLimits[l]["jointUpperLimit"] for l in self.finger_links_withActiveJoints]

        # print("j_values:: ",j_values)
        # print("j_values_upper:: ",j_values_upper)
        # print("self.divition*j_step:: ",[self.divition * step for step in j_step])
        # print("j_step:: ",j_step)
        # print("j_values[1:]:: ",j_values[1:])
        # print("j_values[-1]:: ",j_values[-1])
        # sys.exit()
        # print("j_step::",j_step)

        debug_counter_one = 0
        debug_counter_three = 0
        debug_counter_four = 0
        for i in range(self.divition+1):
            # reseting the other three joints to lower limit
            if i ==0:
                j_values[0] = initial_j_values[0] 
            else:
                j_values[0] += j_step[0]
            
            j_values[1:] =initial_j_values[1:] 
            # reach_able_points.append(self.get_finger_tip_pos(j_values))
            for i in range(self.divition+1):
                if i ==0:
                     j_values[1] = initial_j_values[1]
                else:
                    j_values[1] += j_step[1]
                    
                j_values[2:] =initial_j_values[2:] 
                # reach_able_points.append(self.get_finger_tip_pos(j_values))
                for i in range(self.divition+1):
                    if i ==0:
                         j_values[2] = initial_j_values[2]
                    else:
                        j_values[2] += j_step[2]

                    j_values[3] =initial_j_values[3] 
                    # reach_able_points.append(self.get_finger_tip_pos(j_values))
                  
                    for i in range(self.divition+1):
                        if i ==0:
                            j_values[3] = initial_j_values[3]
                        else:  
                            j_values[3] += j_step[3]

                        reach_able_points.append(self.get_finger_tip_pos(j_values))
        
        return reach_able_points

    def reach_able_points_relative_to_knuckle(self,reachable_points):

        jointInfo = JointInfo()
        jointInfo.get_infoForAll_joints(self.robot)
        active_joints_info  = jointInfo.getActiveJointsInfo()
        num_active_joints = jointInfo.getNumberOfActiveJoints()

        relative_reachable_point = []
        link = self.finger_links[1]
        link_encoded = link.encode(encoding='UTF-8',errors='strict')
        link_joint_info = jointInfo.searchBy(key="linkName",value =link_encoded )[0]
        jointIndx = link_joint_info["jointIndex"]
        # print(link_joint_info)
        knuckle_pose = p.getLinkState(self.robot[0],jointIndx)[0]
        relative_point =[0]*3
        for point in reachable_points:
            relative_point[0] = point[0]-knuckle_pose[0]
            relative_point[1] = point[1]-knuckle_pose[1]
            relative_point[2] = point[2]-knuckle_pose[2]
            relative_reachable_point.append(relative_point)

        return relative_reachable_point

    def create_point_cloud(self,points):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd
        
    def get_finger_tip_pos(self,j_values):
        jointInfo = JointInfo()
        jointInfo.get_infoForAll_joints(self.robot)
        active_joints_info  = jointInfo.getActiveJointsInfo()
        num_active_joints = jointInfo.getNumberOfActiveJoints()
   

        for i,link in enumerate(self.finger_links_withActiveJoints):
            p.resetJointState(self.robot[0],self.jointLimits[link]["jointIndex"],j_values[i])
        

        link = self.finger_links[-1]
        link_encoded = link.encode(encoding='UTF-8',errors='strict')
        link_joint_info = jointInfo.searchBy(key="linkName",value =link_encoded )[0]
        jointIndx = link_joint_info["jointIndex"]
        # print(link_joint_info)
        finger_tip_pos = p.getLinkState(self.robot[0],jointIndx)[0]
        
        # print(finger_tip_pos)

        return list(finger_tip_pos)

    def save_pcd(self,pcd,format="pcd"):
        o3d.io.write_point_cloud(self.finger_name+"."+format, pcd)
        return self.finger_name+"."+format

    def load_pcd(self,path):
        pcd = o3d.io.read_point_cloud(path)
        return pcd
    
    def save_yaml(self,reachable_points):
        reachable_point_dic = {"vertix":reachable_points}
        #dump dic to file
        with io.open(self.finger_name+'.yml', 'w', encoding='utf8') as outfile:
            yaml.dump(reachable_point_dic, outfile)
    
    # def meshFromPcd(self,path):
    #     #https://stackoverflow.com/questions/44997054/point-cloud-to-volume
    #     pcd = PyntCloud.from_file(path)
    #     convex_hull_id = pcd.add_structure("convex_hull")
    #     convex_hull = pcd.structures[convex_hull_id]
    #     pcd.mesh = convex_hull.get_mesh()
    #     pcd.to_file("mesh.ply", also_save=["mesh"])



class Workspace_KUKA():
    def __init__(self,robot,divition=5):
        self.robot = robot
        self.divition = divition
        self.general_links = ["lbr_iiwa_link_"+str(i) for i in range(7+1)]+["lbr_iiwa_link_"+"ee"]
        self.link_with_active_joints = self.general_links[1:-1]   # the first link is constatant lbr_iiwa_link_0
        self.jointLimits = self.get_jointLimits_forLinks(self.link_with_active_joints)
        self.max_in_axis ={
            "x": 0, 
            "y": 0, 
            "z": 0
        }
        self.min_in_axis ={
            "x": 0, 
            "y": 0, 
            "z": 0
        }
   
       

    def get_jointLimits_forLinks(self,links):

        jointInfo = JointInfo()
        jointInfo.get_infoForAll_joints(self.robot)
        active_joints_info  = jointInfo.getActiveJointsInfo()
        num_active_joints = jointInfo.getNumberOfActiveJoints()
        jointLimits = {}
        self.reach_able_points =[]
            
        for link in self.link_with_active_joints:
            link_encoded = link.encode(encoding='UTF-8',errors='strict')
            link_joint_info = jointInfo.searchBy(key="linkName",value =link_encoded )[0]
           
            # print(link+": \n")
            # print(link_joint_info)
            # print("\n")
            jointLimits[link] ={
                "jointIndex":link_joint_info["jointIndex"],
                "jointUpperLimit":link_joint_info["jointUpperLimit"],
                "jointLowerLimit":link_joint_info["jointLowerLimit"],
                "joint_step":(link_joint_info["jointUpperLimit"]- link_joint_info["jointLowerLimit"])/self.divition
            }
  
        return jointLimits
   
    def get_kuka_ee_pos(self,j_values):
        jointInfo = JointInfo()
        jointInfo.get_infoForAll_joints(self.robot)
        active_joints_info  = jointInfo.getActiveJointsInfo()
        num_active_joints = jointInfo.getNumberOfActiveJoints()
   

        for i,link in enumerate(self.link_with_active_joints):
            p.resetJointState(self.robot[0],self.jointLimits[link]["jointIndex"],j_values[i])
        

        ee = self.general_links[-1]
        print("\n\n")
        print("EE::",ee)
    
        link_encoded = ee.encode(encoding='UTF-8',errors='strict')
        link_joint_info = jointInfo.searchBy(key="linkName",value =link_encoded )[0]
        jointIndx = link_joint_info["jointIndex"]
        # print(link_joint_info)
        ee_pos = p.getLinkState(self.robot[0],jointIndx)[0]
        
        print(ee_pos)
        print("\n\n")
        return list(ee_pos)

    def get_reachable_points(self):
        def update_min_max_in_axis(point):
            i=0
            for k,v in self.max_in_axis.items():
                self.max_in_axis[k] = v if v>point[i] else point[i]
                i +=1
            i=0
            for k,v in self.min_in_axis.items():
                self.min_in_axis[k] = v if v<point[i] else point[i]
                i +=1

        def recursive(reach_able_points,j_values,initial_j_values,j_step,loop_counter = 0,num_nested_loops=7):
            if num_nested_loops>=1:
                for i in range(self.divition+1):
                    if i ==0:
                        j_values[7-num_nested_loops] = initial_j_values[7-num_nested_loops] 
                    else:
                        j_values[7-num_nested_loops] += j_step[7-num_nested_loops]

                    j_values[7-num_nested_loops+1:] =initial_j_values[7-num_nested_loops+1:] 
                    loop_counter +=1
            
                    recursive(reach_able_points,j_values,initial_j_values,j_step,loop_counter,num_nested_loops-1)
            else:
   
                point = self.get_kuka_ee_pos(j_values)
                reach_able_points.append(point)
                update_min_max_in_axis(point)

            return reach_able_points

        reach_able_points = []
        j_values = [self.jointLimits[l]["jointLowerLimit"] for l in self.link_with_active_joints]
        initial_j_values = j_values[:]
        j_step = [self.jointLimits[l]["joint_step"] for l in self.link_with_active_joints]

        reach_able_points = recursive(reach_able_points,j_values,initial_j_values,j_step,loop_counter = 0,num_nested_loops=7)

        return reach_able_points
    def create_point_cloud(self,points):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd

    def save_pcd(self,pcd,format="pcd"):
        o3d.io.write_point_cloud("kuka."+format, pcd)
        return "kuka."+format

    def load_pcd(self,path):
        pcd = o3d.io.read_point_cloud(path)
        return pcd
    
    def save_yaml(self,reachable_points):
        reachable_point_dic = {"max_in_axis":self.max_in_axis,
                               "min_in_axis":self.min_in_axis,
                               "vertix":reachable_points
                               }
        #dump dic to file
        with io.open('kuka.yml', 'w', encoding='utf8') as outfile:
            yaml.dump(reachable_point_dic, outfile)
    
ws = Workspace_KUKA(robot)

# reachable_point = ws.get_reachable_points()
# print("reachable_point:: ",reachable_point)
# relative_reachable_point = ws.reach_able_points_relative_to_knuckle(reachable_point)
# print("relative_reachable_point[0]:: ",relative_reachable_point[0])
# pcd = ws.create_point_cloud(reachable_point)
# relative_pcd = ws.create_point_cloud(relative_reachable_point) 
# print("saving point cloud")
# path = ws.save_pcd(pcd,"ply")
# ws.save_yaml(reachable_point)

pcd = ws.load_pcd("kuka.ply")

while(1):
    print("loading the vierwer")
    o3d.visualization.draw_geometries([pcd])
    # p.stepSimulation()