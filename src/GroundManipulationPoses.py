# -*- coding: utf-8 -*-
"""
Created on Fri Apr  6 11:35:28 2018

@author: arturo
"""

import geometry_msgs.msg as geomsg

def get_clear_workspace_lr():

    l_pose = geomsg.Pose()
    l_pose.position.x = 0.35
    l_pose.position.y = 0.55
    l_pose.position.z = 0.21
    l_pose.orientation.x = 0.0
    l_pose.orientation.y = 0.0
    l_pose.orientation.z = 0.0
    l_pose.orientation.w = 1.0

    r_pose = geomsg.Pose()
    r_pose.position.x = 0.35
    r_pose.position.y = -0.55
    r_pose.position.z = 0.21
    r_pose.orientation.x = 0.0
    r_pose.orientation.y = 0.0
    r_pose.orientation.z = 0.0
    r_pose.orientation.w = 1.0
    
    
    return (l_pose, r_pose)
   
   
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
    l_pose.position.x = 0.60
    l_pose.position.y = 0.05
    l_pose.position.z = 0.40
    l_pose.orientation.x = 0.95
    l_pose.orientation.y = 0.30
    l_pose.orientation.z = 0.035
    l_pose.orientation.w = 0.02

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
    l_pose.position.y = 0.20
    l_pose.position.z = 0.95
    l_pose.orientation.x = 0.46
    l_pose.orientation.y = 0.22
    l_pose.orientation.z = 0.50
    l_pose.orientation.w = 0.70

    r_pose = geomsg.Pose()
    r_pose.position.x = 0.65
    r_pose.position.y = -0.20
    r_pose.position.z = 0.95
    r_pose.orientation.x = 0.22
    r_pose.orientation.y = 0.46
    r_pose.orientation.z = 0.70
    r_pose.orientation.w = 0.50
    
    
    return (l_pose, r_pose)    
