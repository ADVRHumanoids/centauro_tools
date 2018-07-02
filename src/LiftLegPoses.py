# -*- coding: utf-8 -*-
"""
Created on Fri Apr  6 11:35:28 2018

@author: arturo
"""

import geometry_msgs.msg as geomsg

def get_reshape_poly_03():

    fl_pose = geomsg.Pose()
    fl_pose.position.x = 0.56
    fl_pose.position.y = 0.14
    fl_pose.position.z = -0.7
    fl_pose.orientation.x = 0.0
    fl_pose.orientation.y = 0.0
    fl_pose.orientation.z = 0.0
    fl_pose.orientation.w = 1.0

  
    rr_pose = geomsg.Pose()
    rr_pose.position.x = -0.30
    rr_pose.position.y = -0.40
    rr_pose.position.z = -0.7
    rr_pose.orientation.x = 0.0
    rr_pose.orientation.y = 0.0
    rr_pose.orientation.z = 0.0
    rr_pose.orientation.w = 1.0
    


    
    
    return (fl_pose, rr_pose)
   
   
