#!/usr/bin/python
# -*- coding: UTF-8 -*-

# Copyright (c). All Rights Reserved.
# -----------------------------------------------------
# File Name:        thmos_walk_engine
# Creator:          JinYin Zhou
# Version:          0.1
# Created:          2023/11/30
# Description:      robot walking control
# History:
#   <author>      <version>       <time>          <description>
#   Jinyin Zhou     0.1           2023/11/30       create
# -----------------------------------------------------

import math
import numpy as np
# from thmos_kinematics import *
from thmos_kinematics_py import *
from thmos_step_planner import *
from thmos_preview_control import *
from thmos_spline import *

""" WALKING STATE
# start walking: double to single, zmp in middle to foot 
# BOOT
# keep walking: single support, zmp in foot
# WALK
# ready to stop: single to double, zmp in foot to middle
# STOP
# stay rested: double support, zmp in middle
# REST
"""

""" SUP_LEG STATE
#LEFT
#RIGHT
"""

class walking():
  def __init__(self, **WalkParams):

    # load params
    for key, value in WalkParams.items():
        setattr(self, key, value)
        
    # period
    self.both_foot_support_frames = self.both_foot_support_time / self.dt
    self.one_step_frames = self.walking_period / self.dt
    self.period_frames = round(self.one_step_frames)
    self.start_up_frame = round(self.both_foot_support_frames)
    
    # init packet
    self.kinematic = thmos_legik(self.way_left, self.way_right, self.leg_rod_length)
    self.spline_smooth = thmos_spline(0,self.foot_height,0,self.period_frames - self.start_up_frame)
    self.mpc_model = thmos_preview_control(self.dt, self.walking_period, self.com_height)
    self.step_plan = thmos_step_planner(self.max_vx, self.max_vy, self.max_vth, self.foot_width)
    
    # init status 
    self.now_x = np.matrix([[0.0], [0.0], [0.0]])
    self.now_y = np.matrix([[0.0], [0.0], [0.0]])
    self.now_vel = [0, 0, 0]
    self.new_vel = [0, 0, 0]
    self.zmp_now = [0,0]
    self.zmp_new = [0,0]
    self.zmp_step = [self.zmp_now,self.zmp_new]
    self.walking_state = 'REST'
    self.support_leg = 'RIGHT'
    self.left_up = self.right_up = 0.0
    self.now_frame = 1

    # leg offset init state
    self.left_off,  self.left_off_g,  self.left_off_d  = np.matrix([[0.0, 0.5*self.foot_width, 0.0]]),  np.matrix([[0.0, 0.0, 0.0]]),  np.matrix([[0.0, 0.0, 0.0]]) 
    self.right_off, self.right_off_g, self.right_off_d = np.matrix([[0.0, -0.5*self.foot_width, 0.0]]),  np.matrix([[0.0, 0.0, 0.0]]),  np.matrix([[0.0, 0.0, 0.0]])
    
    # pos_list - [(com)x, y, theta, (left)x, y, (right)x, y]
    self.pos_list  = [0,0,0, 0, 0.5*self.foot_width, 0, -0.5*self.foot_width]
    self.next_foot_step = [0,0,0, 0, 0.5*self.foot_width, 0, -0.5*self.foot_width]

    return


  def setGoalVel(self, vel = [0,0,0]):
    '''get vel command and be a state machine'''
    self.new_vel = vel.copy()
    self.zmp_now = self.zmp_new.copy()

    # caculate now walking state
    self.walking_state_machine()

    # caculate step list
    self.next_foot_step, self.zmp_new = self.step_plan.step_plan(self.new_vel, self.pos_list, self.support_leg, self.walking_state)
    self.zmp_step = [self.zmp_now,self.zmp_new]
    
    # caculate move goal
    self.left_off_g  = np.matrix([[self.next_foot_step[3], self.next_foot_step[4], self.next_foot_step[2]]])
    self.right_off_g = np.matrix([[self.next_foot_step[5], self.next_foot_step[6], self.next_foot_step[2]]])
    self.left_off_d  = (self.left_off_g - self.left_off) / (self.one_step_frames - self.both_foot_support_frames)
    self.right_off_d = (self.right_off_g - self.right_off) / (self.one_step_frames - self.both_foot_support_frames)

    # update
    self.pos_list = self.next_foot_step.copy()
    self.now_vel = vel.copy()
    
    # loop
    self.now_frame = 1
    return 
      
      
  def getNextPos(self):
    '''set each movement'''
    
    # get this pix move
    x, y = self.mpc_model.CoM_motion(self.now_frame, self.now_x, self.now_y, self.zmp_step)
    self.now_x = x.copy()
    self.now_y = y.copy()
    
    # move foot in the axes of x,y,theta
    if self.now_frame > self.start_up_frame:
       self.right_off += self.right_off_d
       self.left_off  += self.left_off_d
       if self.now_frame > self.period_frames:
         self.right_off = self.right_off_g.copy()
         self.left_off  = self.left_off_g.copy()

    # up and drop foot      
    if self.support_leg == 'LEFT' and self.walking_state != 'REST' and self.walking_state != 'BOOT':
      if self.now_frame > self.start_up_frame:
        self.left_up  = self.spline_smooth.spline_tracker(self.now_frame - self.start_up_frame)
        self.right_up = 0
      else:
        self.left_up  = 0
        self.right_up = 0     
          
    elif self.support_leg == 'RIGHT' and self.walking_state != 'REST' and self.walking_state != 'BOOT':
      if self.now_frame > self.start_up_frame:
        self.left_up  = 0
        self.right_up = self.spline_smooth.spline_tracker(self.now_frame - self.start_up_frame)
      else:
        self.left_up  = 0
        self.right_up = 0        
        
    else:
      pass
        
    # foot theta
    self.th = self.next_foot_step[2]
    
    # caculate foot pos
    lo = self.left_off  - np.block([[x[0,0],y[0,0],0]])
    ro = self.right_off - np.block([[x[0,0],y[0,0],0]])
    
    left_foot  = [lo[0,0] * 0 + self.com_x_offset * 0, lo[0,1] + self.com_y_offset + self.ex_foot_width, self.left_up -  self.trunk_height, 0.0, 0.0, self.th-lo[0,2]]
    right_foot = [ro[0,0] * 0 + self.com_x_offset * 0, ro[0,1] + self.com_y_offset - self.ex_foot_width, self.right_up - self.trunk_height, 0.0, 0.0, self.th-ro[0,2]]

    l_joint_angles = self.kinematic.LegIKMove('left',left_foot)
    r_joint_angles = self.kinematic.LegIKMove('right',right_foot)
    self.joint_angles = r_joint_angles + l_joint_angles  # R angle first
    
    # loop
    self.now_frame += 1

    return self.joint_angles, (self.period_frames - self.now_frame)
  
  def walking_state_machine(self):
    """
    Update the bot walking status.
    """
    if_vold_zero = (self.now_vel[0] == 0 and self.now_vel[1] == 0 and self.now_vel[2] == 0)
    if_vnew_zero = (self.new_vel[0] == 0 and self.new_vel[1] == 0 and self.new_vel[2] == 0)
    
    if(if_vold_zero):
      if(if_vnew_zero):        
        self.walking_state = 'REST'
      else:
        self.walking_state = 'BOOT'
    else:
      if(if_vnew_zero):
        self.walking_state = 'STOP'
      else:     
        self.walking_state = 'WALK'

    if(self.support_leg == 'RIGHT'):
      self.support_leg = 'LEFT'
    else:
      self.support_leg = 'RIGHT'
    return

    
if __name__ == '__main__':
    pass
