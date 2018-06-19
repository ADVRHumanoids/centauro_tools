# -*- coding: utf-8 -*-
"""
Created on Fri Apr  6 11:35:28 2018

@author: arturo
"""

import geometry_msgs.msg as geomsg

def get_up_stretched_0123wa():

    fl_pose = geomsg.Pose()
    fl_pose.position.x = 0.35
    fl_pose.position.y = 0.35
    fl_pose.position.z = -0.7
    fl_pose.orientation.x = 0.0
    fl_pose.orientation.y = 0.0
    fl_pose.orientation.z = 0.0
    fl_pose.orientation.w = 1.0
    
    fr_pose = geomsg.Pose()
    fr_pose.position.x = 0.35
    fr_pose.position.y = -0.35
    fr_pose.position.z = -0.7
    fr_pose.orientation.x = 0.0
    fr_pose.orientation.y = 0.0
    fr_pose.orientation.z = 0.0
    fr_pose.orientation.w = 1.0
    
    rr_pose = geomsg.Pose()
    rr_pose.position.x = -0.35
    rr_pose.position.y = -0.35
    rr_pose.position.z = -0.7
    rr_pose.orientation.x = 0.0
    rr_pose.orientation.y = 0.0
    rr_pose.orientation.z = 0.0
    rr_pose.orientation.w = 1.0
    
    rl_pose = geomsg.Pose()
    rl_pose.position.x = -0.35
    rl_pose.position.y = 0.35
    rl_pose.position.z = -0.7
    rl_pose.orientation.x = 0.0
    rl_pose.orientation.y = 0.0
    rl_pose.orientation.z = 0.0
    rl_pose.orientation.w = 1.0
    
    waist = geomsg.Pose()
    waist.position.x = 0.0
    waist.position.y = 0.0
    waist.position.z = 0.95
    waist.orientation.x = 0.0
    waist.orientation.y = 0.0
    waist.orientation.z = 0.0
    waist.orientation.w = 1.0


    
    
    return (fl_pose, fr_pose, rr_pose, rl_pose, waist)
   
   

def get_down_mid_0123wa():

    fl_pose = geomsg.Pose()
    fl_pose.position.x = 0.35
    fl_pose.position.y = 0.45
    fl_pose.position.z = -0.7
    fl_pose.orientation.x = 0.0
    fl_pose.orientation.y = 0.0
    fl_pose.orientation.z = 0.0
    fl_pose.orientation.w = 1.0
    
    fr_pose = geomsg.Pose()
    fr_pose.position.x = 0.35
    fr_pose.position.y = -0.45
    fr_pose.position.z = -0.7
    fr_pose.orientation.x = 0.0
    fr_pose.orientation.y = 0.0
    fr_pose.orientation.z = 0.0
    fr_pose.orientation.w = 1.0
    
    rr_pose = geomsg.Pose()
    rr_pose.position.x = -0.35
    rr_pose.position.y = -0.45
    rr_pose.position.z = -0.7
    rr_pose.orientation.x = 0.0
    rr_pose.orientation.y = 0.0
    rr_pose.orientation.z = 0.0
    rr_pose.orientation.w = 1.0
    
    rl_pose = geomsg.Pose()
    rl_pose.position.x = -0.35
    rl_pose.position.y = 0.45
    rl_pose.position.z = -0.7
    rl_pose.orientation.x = 0.0
    rl_pose.orientation.y = 0.0
    rl_pose.orientation.z = 0.0
    rl_pose.orientation.w = 1.0
    
    waist = geomsg.Pose()
    waist.position.x = 0.0
    waist.position.y = 0.0
    waist.position.z = 0.75
    waist.orientation.x = 0.0
    waist.orientation.y = 0.0
    waist.orientation.z = 0.0
    waist.orientation.w = 1.0


    
    
    return (fl_pose, fr_pose, rr_pose, rl_pose, waist)
   
   
   
def get_down_low_0123wa():

    fl_pose = geomsg.Pose()
    fl_pose.position.x = 0.35
    fl_pose.position.y = 0.55
    fl_pose.position.z = -0.7
    fl_pose.orientation.x = 0.0
    fl_pose.orientation.y = 0.0
    fl_pose.orientation.z = 0.0
    fl_pose.orientation.w = 1.0
    
    fr_pose = geomsg.Pose()
    fr_pose.position.x = 0.35
    fr_pose.position.y = -0.55
    fr_pose.position.z = -0.7
    fr_pose.orientation.x = 0.0
    fr_pose.orientation.y = 0.0
    fr_pose.orientation.z = 0.0
    fr_pose.orientation.w = 1.0
    
    rr_pose = geomsg.Pose()
    rr_pose.position.x = -0.35
    rr_pose.position.y = -0.55
    rr_pose.position.z = -0.7
    rr_pose.orientation.x = 0.0
    rr_pose.orientation.y = 0.0
    rr_pose.orientation.z = 0.0
    rr_pose.orientation.w = 1.0
    
    rl_pose = geomsg.Pose()
    rl_pose.position.x = -0.35
    rl_pose.position.y = 0.55
    rl_pose.position.z = -0.7
    rl_pose.orientation.x = 0.0
    rl_pose.orientation.y = 0.0
    rl_pose.orientation.z = 0.0
    rl_pose.orientation.w = 1.0
    
    waist = geomsg.Pose()
    waist.position.x = 0.0
    waist.position.y = 0.0
    waist.position.z = 0.55
    waist.orientation.x = 0.0
    waist.orientation.y = 0.0
    waist.orientation.z = 0.0
    waist.orientation.w = 1.0


    
    
    return (fl_pose, fr_pose, rr_pose, rl_pose, waist)   
   
def get_wheels_home_lr():

    l_pose = geomsg.Pose()
    l_pose.position.x = 0.50
    l_pose.position.y = 0.22
    l_pose.position.z = 0.21
    l_pose.orientation.x = 0.0
    l_pose.orientation.y = 0.0
    l_pose.orientation.z = 0.0
    l_pose.orientation.w = 1.0

    r_pose = geomsg.Pose()
    r_pose.position.x = 0.50
    r_pose.position.y = -0.22
    r_pose.position.z = 0.21
    r_pose.orientation.x = 0.0
    r_pose.orientation.y = 0.0
    r_pose.orientation.z = 0.0
    r_pose.orientation.w = 1.0
    
    
    return (l_pose, r_pose)
    
def get_down_open_lr():

    l_pose = geomsg.Pose()
    l_pose.position.x = 0.95
    l_pose.position.y = 0.15
    l_pose.position.z = 0.30
    l_pose.orientation.x = 0.6
    l_pose.orientation.y = 0.6
    l_pose.orientation.z = 0.4
    l_pose.orientation.w = 0.4

    r_pose = geomsg.Pose()
    r_pose.position.x = 0.95
    r_pose.position.y = -0.15
    r_pose.position.z = 0.30
    r_pose.orientation.x = 0.5
    r_pose.orientation.y = 0.6
    r_pose.orientation.z = 0.4
    r_pose.orientation.w = 0.4
    
    
    return (l_pose, r_pose)


def get_up_close_lr():


    l_pose = geomsg.Pose()
    l_pose.position.x = 0.65
    l_pose.position.y = 0.15
    l_pose.position.z = 0.95
    l_pose.orientation.x = 0.3
    l_pose.orientation.y = 0.4
    l_pose.orientation.z = 0.6
    l_pose.orientation.w = 0.6

    r_pose = geomsg.Pose()
    r_pose.position.x = 0.65
    r_pose.position.y = -0.15
    r_pose.position.z = 0.95
    r_pose.orientation.x = 0.3
    r_pose.orientation.y = 0.4
    r_pose.orientation.z = 0.6
    r_pose.orientation.w = 0.6
    
    
    return (l_pose, r_pose)    
