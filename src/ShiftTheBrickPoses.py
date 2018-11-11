# -*- coding: utf-8 -*-
"""
Created on Fri Apr  6 11:35:28 2018

@author: arturo
"""

import geometry_msgs.msg as geomsg

def get_down_stable():

    fl_pose = geomsg.Pose()
    fl_pose.position.x = 0.50
    fl_pose.position.y = 0.33
    fl_pose.position.z = -0.7
    fl_pose.orientation.x = 0.0
    fl_pose.orientation.y = 0.0
    fl_pose.orientation.z = 0.0
    fl_pose.orientation.w = 1.0
    
    fr_pose = geomsg.Pose()
    fr_pose.position.x = 0.36
    fr_pose.position.y = -0.22
    fr_pose.position.z = -0.7
    fr_pose.orientation.x = 0.0
    fr_pose.orientation.y = 0.0
    fr_pose.orientation.z = 0.0
    fr_pose.orientation.w = 1.0
    
    rr_pose = geomsg.Pose()
    rr_pose.position.x = -0.00
    rr_pose.position.y = -0.35
    rr_pose.position.z = -0.7
    rr_pose.orientation.x = 0.0
    rr_pose.orientation.y = 0.0
    rr_pose.orientation.z = 0.0
    rr_pose.orientation.w = 1.0
    
    rl_pose = geomsg.Pose()
    rl_pose.position.x = -0.25
    rl_pose.position.y = 0.32
    rl_pose.position.z = -0.7
    rl_pose.orientation.x = 0.0
    rl_pose.orientation.y = 0.0
    rl_pose.orientation.z = 0.0
    rl_pose.orientation.w = 1.0
    
    waist = geomsg.Pose()
    waist.position.x = 0.0
    waist.position.y = 0.0
    waist.position.z = 0.80
    waist.orientation.x = 0.0
    waist.orientation.y = 0.0
    waist.orientation.z = 0.0
    waist.orientation.w = 1.0


    
    
    return (fl_pose, fr_pose, rr_pose, rl_pose, waist)
   
   
def get_push_brick_fr():

    fr_pose = geomsg.Pose()
    fr_pose.position.x = 0.58
    fr_pose.position.y = -0.42
    fr_pose.position.z = -0.7
    fr_pose.orientation.x = 0.0
    fr_pose.orientation.y = 0.0
    fr_pose.orientation.z = 0.0
    fr_pose.orientation.w = 1.0   
    
    return fr_pose



def get_homing():

    fl_pose = geomsg.Pose()
    fl_pose.position.x = 0.50
    fl_pose.position.y = 0.22
    fl_pose.position.z = -0.7
    fl_pose.orientation.x = 0.0
    fl_pose.orientation.y = 0.0
    fl_pose.orientation.z = 0.0
    fl_pose.orientation.w = 1.0
    
    fr_pose = geomsg.Pose()
    fr_pose.position.x = 0.50
    fr_pose.position.y = -0.22
    fr_pose.position.z = -0.7
    fr_pose.orientation.x = 0.0
    fr_pose.orientation.y = 0.0
    fr_pose.orientation.z = 0.0
    fr_pose.orientation.w = 1.0
    
    rr_pose = geomsg.Pose()
    rr_pose.position.x = -0.50
    rr_pose.position.y = -0.22
    rr_pose.position.z = -0.7
    rr_pose.orientation.x = 0.0
    rr_pose.orientation.y = 0.0
    rr_pose.orientation.z = 0.0
    rr_pose.orientation.w = 1.0
    
    rl_pose = geomsg.Pose()
    rl_pose.position.x = -0.50
    rl_pose.position.y = 0.22
    rl_pose.position.z = -0.7
    rl_pose.orientation.x = 0.0
    rl_pose.orientation.y = 0.0
    rl_pose.orientation.z = 0.0
    rl_pose.orientation.w = 1.0
    
    waist = geomsg.Pose()
    waist.position.x = 0.0
    waist.position.y = 0.0
    waist.position.z = 0.90
    waist.orientation.x = 0.0
    waist.orientation.y = 0.0
    waist.orientation.z = 0.0
    waist.orientation.w = 1.0
    
    return (fl_pose, fr_pose, rr_pose, rl_pose, waist)