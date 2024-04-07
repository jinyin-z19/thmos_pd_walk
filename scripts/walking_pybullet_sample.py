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
  TIME_STEP = 0.0001
  physicsClient = p.connect(p.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.setGravity(0, 0, -9.8)
  p.setTimeStep(TIME_STEP)

  planeId = p.loadURDF("plane.urdf", [0, 0, 0])
  RobotId = p.loadURDF("../urdf/urdf/thmos_mix.urdf", [0, 0, 0.43],useFixedBase = False)  #0.43
	

  index = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
  for id in range(p.getNumJoints(RobotId)):
    index[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = id
    link_name = p.getJointInfo(RobotId, id)[12].decode('UTF-8')
    print([id,link_name])

  joint_angles = []
  for id in range(p.getNumJoints(RobotId)):
    if p.getJointInfo(RobotId, id)[3] > -1:
      joint_angles += [0,]

  index_dof = {p.getBodyInfo(RobotId)[0].decode('UTF-8'):-1,}
  for id in range(p.getNumJoints(RobotId)):
    index_dof[p.getJointInfo(RobotId, id)[12].decode('UTF-8')] = p.getJointInfo(RobotId, id)[3] - 7

  p.changeDynamics(planeId, -1, lateralFriction=10000000)
  p.changeDynamics(RobotId, index['r_sole'], lateralFriction=1000000)
  p.changeDynamics(RobotId, index['l_sole'], lateralFriction=1000000)
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
            'k_y_offset':param[14],#ex_com_y_offset k
            'trunk_pitch':param[15],
            'way_left' : [1,-1,-1,-1,-1,-1],
            'way_right' : [1,1,-1,1,1,-1],
            'leg_rod_length' : [0.156,0.12,0.045]
            }

  walk = walking(**Params)
  j = 0
  n = 0
  k = 0 
  nk = 0
  zmp_x_s = 100 * [0]
  zmp_y_s = 100 * [0]
  zmp_x = 0
  zmp_y = 0
  acc = [0,0]
  base_pos = [0,0]
  while p.isConnected():
    zmp_x_s[j] =  - Params['com_height'] / 9.8 * acc[0]
    zmp_y_s[j] =  - Params['com_height'] / 9.8 * acc[1]
    j += 1
    
    if j >= 100:
      zmp_x_n = base_pos[0] + np.mean(zmp_x_s)
      zmp_y_n = base_pos[1] + np.mean(zmp_y_s)
      
      p.addUserDebugLine([zmp_x_n,zmp_y_n,0.0], [zmp_x_n,zmp_y_n,0.1], lineColorRGB=[1, 0, 0], lifeTime = 1, lineWidth = 3)
      zmp_x = zmp_x_n
      zmp_y = zmp_y_n 
      if n == 0:
        if nk < 8:
          walk.setGoalVel([(random()-0.5)*0.3 * 0 + 0.0, (random()-0.5)*0.3 * 0 + 0.2, (random()-0.5)*0.2 * 0])
          nk = nk + 1
        elif nk < 12:
          walk.setGoalVel([(random()-0.5)*0.3 * 0 + 0.0, (random()-0.5)*0.3 * 0 + 0.2, (random()-0.5)*0.2 * 0])
          nk = nk + 1
        else:
          walk.setGoalVel([(random()-0.5)*0.3 * 0 + 0.0, (random()-0.5)*0.3 * 0 + 0.2, (random()-0.5)*0.2 * 0])
          nk = 0
      joint_angles,n = walk.getNextPos()
      j = 0
    
    for id in range(p.getNumJoints(RobotId)):
      qIndex = p.getJointInfo(RobotId, id)[3]
      if qIndex > -1:
        if 'leg' in p.getJointInfo(RobotId, id)[1].decode('UTF-8'):   # R_leg_1 to L_leg_6: 15-26
          if '1' in p.getJointInfo(RobotId, id)[1].decode('UTF-8'):   # hip   MX106
            p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-15], force = 6) 
          elif '6' in p.getJointInfo(RobotId, id)[1].decode('UTF-8'): # ankle MX106
            p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-15], force = 6)
          else: # 64
            p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-15], force = 4)
        else:
          p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, 0, force=100)
          
    # caculate sensor
    v_old,_ = p.getBaseVelocity(RobotId)
    p.stepSimulation()
    v_new,_ = p.getBaseVelocity(RobotId)
    base_pos,_ = p.getBasePositionAndOrientation(RobotId)
    acc = (np.array(v_new) - np.array(v_old)) / TIME_STEP

