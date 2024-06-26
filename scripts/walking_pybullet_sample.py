#!/usr/bin/env python3
import pybullet as p
import pybullet_data
import numpy as np
import sys
sys.path.append('./walking_packet')
from thmos_walk_engine import *
from random import random 
from scipy.spatial.transform import Rotation as R

      
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
  #zmp_x_s = 100 * [0]
  #zmp_y_s = 100 * [0]
  #zmp_x = 0
  #zmp_y = 0
  #acc = [0,0]
  #base_pos = [0,0]
  roll_ang = 0
  pitch_ang = 0
  while p.isConnected():
    #zmp_x_s[j] =  - Params['com_height'] / 9.8 * acc[0]
    #zmp_y_s[j] =  - Params['com_height'] / 9.8 * acc[1]
    j += 1
    
    if j >= 10:
      #zmp_x = base_pos[0] + np.mean(zmp_x_s)
      #zmp_y = base_pos[1] + np.mean(zmp_y_s)
      #if n < round(0.6 *  Params['walking_period'] / Params['dt']):
        #p.addUserDebugLine([zmp_x,zmp_y,0.0], [zmp_x,zmp_y,0.1], lineColorRGB=[1, 0, 0], lifeTime = 1, lineWidth = 3)
      #else:
        #p.addUserDebugLine([zmp_x,zmp_y,0.0], [zmp_x,zmp_y,0.1], lineColorRGB=[0, 1, 0], lifeTime = 1, lineWidth = 3)

      # paint ang
      p.addUserDebugLine([base_pos[0],base_pos[1],base_pos[2]], 
                         [base_pos[0] +  0.2 * base_vec[0],
                          base_pos[1] +  0.2 * base_vec[1],
                          base_pos[2] +  0.2 * base_vec[2]], lineColorRGB=[1, 0, 0], lifeTime = 0.02, lineWidth = 3)
      p.addUserDebugLine([base_pos[0] * 0 ,base_pos[1] * 0,base_pos[2] * 0], 
                         [base_pos[0] * 0 +  1 * base_vec2[0],
                          base_pos[1] * 0 +  1 * base_vec2[1],
                          base_pos[2] * 0 +  1 * base_vec2[2]], lineColorRGB=[0, 0, 1], lifeTime = 0.02, lineWidth = 3)
      
      # ankle feed back
      rfb = 0.24 * roll_ang  + 0.06 * roll_speed
      pfb = 0.24 * pitch_ang + 0.06 * pitch_speed

      #else:
      if n == 0:
        if nk < 8:
          walk.setGoalVel([(random()-0.5)*0.3 * 0 + 0.15, (random()-0.5)*0.3 * 0 + 0.0, (random()-0.5)*0.2 * 0])
          nk = nk + 1
        elif nk < 12:
          walk.setGoalVel([(random()-0.5)*0.3 * 0 + 0.15, (random()-0.5)*0.3 * 0 + 0.0, (random()-0.5)*0.2 * 0])
          nk = nk + 1
        else:
          walk.setGoalVel([(random()-0.5)*0.3 * 0 + 0.15, (random()-0.5)*0.3 * 0 + 0.0, (random()-0.5)*0.2 * 0])
          nk = 0
      joint_angles,n = walk.getNextPos(rfb ,pfb )
      j = 0
    
    for id in range(p.getNumJoints(RobotId)):
      qIndex = p.getJointInfo(RobotId, id)[3]
      if qIndex > -1:
        if 'leg' in p.getJointInfo(RobotId, id)[1].decode('UTF-8'):   # R_leg_1 to L_leg_6: 15-26
          if '1' in p.getJointInfo(RobotId, id)[1].decode('UTF-8'):   # hip   MX106
            p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-15], force = 8) 
          elif '6' in p.getJointInfo(RobotId, id)[1].decode('UTF-8'): # ankle MX106
            p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-15], force = 8)
          else: # 64
            p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, joint_angles[qIndex-15], force = 6)
        else:
          p.setJointMotorControl2(RobotId, id, p.POSITION_CONTROL, 0, force=100)
          
    # caculate sensor
    v_old,_ = p.getBaseVelocity(RobotId)
    p.stepSimulation()
    v_new,v_ang = p.getBaseVelocity(RobotId)
    base_pos,base_qua = p.getBasePositionAndOrientation(RobotId)

    # caculate body vec
    r = R.from_quat(base_qua)
    rot = r.as_matrix()
    base_vec = rot[0:3,2]   

    # caculate sensor rp
    base_euler = r.as_euler('zyx',degrees=False)
    base_euler[0] = 0
    r2 = R.from_euler('zyx',base_euler,degrees=False)
    rot2 = r2.as_matrix()
    base_vec2 = rot2[0:3,2]

    roll_speed = (base_euler[2] -  roll_ang) / TIME_STEP
    pitch_speed = (base_euler[1] -  pitch_ang) / TIME_STEP

    roll_ang = base_euler[2]
    pitch_ang = base_euler[1]

    #acc = (np.array(v_new) - np.array(v_old)) / TIME_STEP

