# -*- coding: utf-8 -*-
"""
Created on Fri Apr  6 11:35:28 2018

@author: arturo
"""

import geometry_msgs.msg as geomsg


def get_reshape_poly_home():

    fl_pose = geomsg.Pose()
    fl_pose.position.x = 0.55
    fl_pose.position.y = 0.22
    fl_pose.position.z = -0.7
    fl_pose.orientation.x = 0.0
    fl_pose.orientation.y = 0.0
    fl_pose.orientation.z = 0.0
    fl_pose.orientation.w = 1.0

    fr_pose = geomsg.Pose()
    fr_pose.position.x = 0.55
    fr_pose.position.y = -0.22
    fr_pose.position.z = -0.7
    fr_pose.orientation.x = 0.0
    fr_pose.orientation.y = 0.0
    fr_pose.orientation.z = 0.0
    fr_pose.orientation.w = 1.0
    
    rl_pose = geomsg.Pose()
    rl_pose.position.x = -0.55
    rl_pose.position.y = 0.22
    rl_pose.position.z = -0.7
    rl_pose.orientation.x = 0.0
    rl_pose.orientation.y = 0.0
    rl_pose.orientation.z = 0.0
    rl_pose.orientation.w = 1.0
  
    rr_pose = geomsg.Pose()
    rr_pose.position.x = -0.55
    rr_pose.position.y = -0.22
    rr_pose.position.z = -0.7
    rr_pose.orientation.x = 0.0
    rr_pose.orientation.y = 0.0
    rr_pose.orientation.z = 0.0
    rr_pose.orientation.w = 1.0
    

    return (fl_pose, fr_pose, rr_pose, rl_pose)
    
    
def get_reshape_poly_fr():

    fl_pose = geomsg.Pose()
    fl_pose.position.x = 0.56
    fl_pose.position.y = 0.14
    fl_pose.position.z = -0.7
    fl_pose.orientation.x = 0.0
    fl_pose.orientation.y = 0.0
    fl_pose.orientation.z = 0.0
    fl_pose.orientation.w = 1.0

    fr_pose = geomsg.Pose()
    fr_pose.position.x = 0.55
    fr_pose.position.y = -0.22
    fr_pose.position.z = -0.7
    fr_pose.orientation.x = 0.0
    fr_pose.orientation.y = 0.0
    fr_pose.orientation.z = 0.0
    fr_pose.orientation.w = 1.0
    
    rl_pose = geomsg.Pose()
    rl_pose.position.x = -0.55
    rl_pose.position.y = 0.22
    rl_pose.position.z = -0.7
    rl_pose.orientation.x = 0.0
    rl_pose.orientation.y = 0.0
    rl_pose.orientation.z = 0.0
    rl_pose.orientation.w = 1.0
  
    rr_pose = geomsg.Pose()
    rr_pose.position.x = -0.30
    rr_pose.position.y = -0.40
    rr_pose.position.z = -0.7
    rr_pose.orientation.x = 0.0
    rr_pose.orientation.y = 0.0
    rr_pose.orientation.z = 0.0
    rr_pose.orientation.w = 1.0
    

    return (fl_pose, fr_pose, rr_pose, rl_pose)
   
   
def get_reshape_poly_rl():

    fl_pose = geomsg.Pose()
    fl_pose.position.x = 0.29
    fl_pose.position.y = 0.36
    fl_pose.position.z = -0.7
    fl_pose.orientation.x = 0.0
    fl_pose.orientation.y = 0.0
    fl_pose.orientation.z = 0.0
    fl_pose.orientation.w = 1.0

    fr_pose = geomsg.Pose()
    fr_pose.position.x = 0.55
    fr_pose.position.y = -0.22
    fr_pose.position.z = -0.7
    fr_pose.orientation.x = 0.0
    fr_pose.orientation.y = 0.0
    fr_pose.orientation.z = 0.0
    fr_pose.orientation.w = 1.0

    rl_pose = geomsg.Pose()
    rl_pose.position.x = -0.55
    rl_pose.position.y = 0.22
    rl_pose.position.z = -0.7
    rl_pose.orientation.x = 0.0
    rl_pose.orientation.y = 0.0
    rl_pose.orientation.z = 0.0
    rl_pose.orientation.w = 1.0

    rr_pose = geomsg.Pose()
    rr_pose.position.x = -0.55
    rr_pose.position.y = -0.22
    rr_pose.position.z = -0.7
    rr_pose.orientation.x = 0.0
    rr_pose.orientation.y = 0.0
    rr_pose.orientation.z = 0.0
    rr_pose.orientation.w = 1.0


    return (fl_pose, fr_pose, rr_pose, rl_pose)
   
   
def get_reshape_poly_rr():

    fl_pose = geomsg.Pose()
    fl_pose.position.x = 0.55
    fl_pose.position.y = 0.22
    fl_pose.position.z = -0.7
    fl_pose.orientation.x = 0.0
    fl_pose.orientation.y = 0.0
    fl_pose.orientation.z = 0.0
    fl_pose.orientation.w = 1.0

    fr_pose = geomsg.Pose()
    fr_pose.position.x = 0.29
    fr_pose.position.y = -0.36
    fr_pose.position.z = -0.7
    fr_pose.orientation.x = 0.0
    fr_pose.orientation.y = 0.0
    fr_pose.orientation.z = 0.0
    fr_pose.orientation.w = 1.0

    rl_pose = geomsg.Pose()
    rl_pose.position.x = -0.55
    rl_pose.position.y = 0.22
    rl_pose.position.z = -0.7
    rl_pose.orientation.x = 0.0
    rl_pose.orientation.y = 0.0
    rl_pose.orientation.z = 0.0
    rl_pose.orientation.w = 1.0

    rr_pose = geomsg.Pose()
    rr_pose.position.x = -0.55
    rr_pose.position.y = -0.22
    rr_pose.position.z = -0.7
    rr_pose.orientation.x = 0.0
    rr_pose.orientation.y = 0.0
    rr_pose.orientation.z = 0.0
    rr_pose.orientation.w = 1.0


    return (fl_pose, fr_pose, rr_pose, rl_pose)

def get_reshape_poly_fl():

    fl_pose = geomsg.Pose()
    fl_pose.position.x = 0.55
    fl_pose.position.y = 0.22
    fl_pose.position.z = -0.7
    fl_pose.orientation.x = 0.0
    fl_pose.orientation.y = 0.0
    fl_pose.orientation.z = 0.0
    fl_pose.orientation.w = 1.0

    fr_pose = geomsg.Pose()
    fr_pose.position.x = 0.56
    fr_pose.position.y = -0.14
    fr_pose.position.z = -0.7
    fr_pose.orientation.x = 0.0
    fr_pose.orientation.y = 0.0
    fr_pose.orientation.z = 0.0
    fr_pose.orientation.w = 1.0
    
    rl_pose = geomsg.Pose()
    rl_pose.position.x = -0.30
    rl_pose.position.y = 0.40
    rl_pose.position.z = -0.7
    rl_pose.orientation.x = 0.0
    rl_pose.orientation.y = 0.0
    rl_pose.orientation.z = 0.0
    rl_pose.orientation.w = 1.0
  
    rr_pose = geomsg.Pose()
    rr_pose.position.x = -0.55
    rr_pose.position.y = -0.22
    rr_pose.position.z = -0.7
    rr_pose.orientation.x = 0.0
    rr_pose.orientation.y = 0.0
    rr_pose.orientation.z = 0.0
    rr_pose.orientation.w = 1.0
    

    return (fl_pose, fr_pose, rr_pose, rl_pose)