#!/usr/bin/env python


import rospy
import smach
import smach_ros
from cartesian_interface import pyci
from cartesian_interface import roscpp_utils as roscpp
from cartesian_interface.affine3 import Affine3
import sys
import threading
import numpy
import time

if __name__ == '__main__':

    node_name = 'stable_support'
    roscpp.init(node_name, sys.argv)    
    rospy.init_node(node_name, sys.argv)
        
    robot = pyci.CartesianInterfaceRos()
    
    fl_wheel = 'wheel_1'
    fr_wheel = 'wheel_2'
    rr_wheel = 'wheel_4'
    rl_wheel = 'wheel_3'
    fl_ankle = 'ankle2_1'
    fr_ankle = 'ankle2_2'
    rr_ankle = 'ankle2_4'
    rl_ankle = 'ankle2_3'
    waist = 'pelvis'
    
    fl_sgn = numpy.array([ 1,  1, 1])
    fr_sgn = numpy.array([ 1, -1, 1])
    rr_sgn = numpy.array([-1, -1, 1])
    rl_sgn = numpy.array([-1,  1, 1])
    
    fl = Affine3(pos=[ 0.35,  0.35, -0.70])
    fr = Affine3(pos=[ 0.35, -0.35, -0.70])
    rr = Affine3(pos=[-0.35, -0.35, -0.70])
    rl = Affine3(pos=[-0.35,  0.35, -0.70])
    wa = Affine3(pos=[0.0, 0.0, 0.82], rot=[0, 0, 0, 1])
    
    robot.setTargetPose(fl_wheel, fl, 5.0)
    robot.setTargetPose(fr_wheel, fr, 5.0)
    robot.setTargetPose(rr_wheel, rr, 5.0)
    robot.setTargetPose(rl_wheel, rl, 5.0)

    robot.waitReachCompleted(fl_wheel)
    robot.waitReachCompleted(fr_wheel)
    robot.waitReachCompleted(rr_wheel)
    robot.waitReachCompleted(rl_wheel)
    
    print 'Exiting..'

