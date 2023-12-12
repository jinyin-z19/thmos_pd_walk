#!/usr/bin/env python3
import pybullet as p
import pybullet_data
import numpy as np
import sys
sys.path.append('./walking_packet')
from thmos_walk_engine import *
from random import random 
from time import sleep
import time

if __name__ == '__main__':
  TIME_STEP = 0.001
  physicsClient = p.connect(p.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.setGravity(0, 0, -9.8)
  p.setTimeStep(TIME_STEP)

  planeId = p.loadURDF("plane.urdf", [0, 0, 0])
  RobotId = p.loadURDF("../urdf/urdf/thmos_mix.urdf", [0, 0, 0.43],useFixedBase = False)  #0.43


  index = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
  for id in range(p.getNumJoints(RobotId)):
    index[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = id

  joint_angles = []
  for id in range(p.getNumJoints(RobotId)):
    if p.getJointInfo(RobotId, id)[3] > -1:
      joint_angles += [0,]

  index_dof = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
  for id in range(p.getNumJoints(RobotId)):
    index_dof[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = p.getJointInfo(RobotId, id)[3] - 7

  # control box ----
  sys.path.append(sys.path[0] + '/param.txt')
  param_path=sys.path[-1]		
  param=np.genfromtxt(fname=param_path,dtype=float,delimiter=",",comments="#",max_rows=38,invalid_raise=False)
  Params = {              
            'foot_width' : param[0],
            'ex_foot_width' : param[1],
            'foot_height' :param[2],
            'com_height' : param[3],
            'com_x_offset' : param[4],
            'com_y_offset' :param[5],
            'trunk_height' : param[6],
            'walking_period' : param[7],
            'both_foot_support_time' : param[8],
            'dt' : param[9],
            'max_vx' : param[10],
            'max_vy': param[11],
            'max_vth' : param[12],
            'k_x_offset':param[13],#ex_com_x_offset k
            'b_x_offset':param[14],#ex_com_x_offset b
            'k_y_offset':param[15],#ex_com_y_offset k
            'b_y_offset':param[16],#ex_com_y_offset b
            'way_left' : [1,-1,-1,-1,-1,-1],
            'way_right' : [1,1,-1,1,1,-1],
            'leg_rod_length' : [0.156,0.12,0.045]
            }

  walk = walking(**Params)
  j = 0
  n = 0
  k = 0 
  nk = 0
  
  while p.isConnected():
    j += 1
    if j >= 10:
      if n == 0:
        if nk < 8:
          walk.setGoalVel([(random()-0.5)*0.3, (random()-0.5)*0.3, (random()-0.5)*0.3])
          nk = nk + 1
        elif nk < 12:
          walk.setGoalVel([(random()-0.5)*0.3, (random()-0.5)*0.3, (random()-0.5)*0.3])
          nk = nk + 1
        else:
          walk.setGoalVel([(random()-0.5)*0.3, (random()-0.5)*0.3, (random()-0.5)*0.3])
          nk = 0
      joint_angles,n = walk.getNextPos()
      j = 0
    
    for id in range(p.getNumJoints(RobotId)):
      qIndex = p.getJointInfo(RobotId, id)[3]
      if qIndex > -1:
        if 'leg' in p.getJointInfo(RobotId, id)[1].decode('UTF-8'):
          p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-15], force=8.5) # R_leg_1 to L_leg_6: 15-26
    p.stepSimulation()

