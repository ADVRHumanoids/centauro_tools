#!/usr/bin/env python


import rospy
import geometry_msgs.msg as geomsg
import std_srvs.srv as stdsrv


def main():
    
    rospy.init_node('walking_control')
    
    rospy.wait_for_service('/make_step')

    raw_input('Enter to start walking..')
    
    try:
        start_walk = rospy.ServiceProxy('/start', stdsrv.SetBool)
        
        res = start_walk(True)
        print res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    


if __name__ == '__main__':
    main()