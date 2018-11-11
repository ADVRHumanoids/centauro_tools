#!/usr/bin/env python


import rospy
import smach
import smach_ros
import CentauroCartesianControl 
import ShiftTheBrickPoses as poses
import geometry_msgs.msg as geomsg

robot = CentauroCartesianControl.CentauroControl()



class GetStable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GET STABLE')
        
        robot.load_controller('WheeledMotion')
        
        # CALL ACTION TO GO DOWN AND WAIT FOR RESULT
        (fl, fr, rr, rl, waist) = poses.get_down_stable()

        
        robot.go_to('fl_wheel', fl, 5.0)
        robot.go_to('fr_wheel', fr, 5.0)
        robot.go_to('hr_wheel', rr, 5.0)
        robot.go_to('hl_wheel', rl, 5.0)
        robot.go_to('waist', waist, 5.0)
        
        robot.wait_for_result('waist')
        robot.wait_for_result('fl_wheel')
        robot.wait_for_result('fr_wheel')
        robot.wait_for_result('hr_wheel')
        robot.wait_for_result('hl_wheel')

        
        return 'done'

class PushBrick(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        
        rospy.loginfo('Executing state PUSH BRICK')
        raw_input('Enter to push the brick')
        robot.load_controller('OpenSot')
        
        
        # CALL ACTION TO GO DOWN AND WAIT FOR RESULT
        #fr = poses.get_push_brick_fr()
        
        fr_delta = geomsg.Pose()
        fr_delta.position.x = 0.0
        fr_delta.position.y = 0.0
        fr_delta.position.z = 0.05

        robot.go_to('fr_wheel', fr_delta, 2.0, True)
        robot.wait_for_result('fr_wheel')
        
        fr_delta = geomsg.Pose()
        fr_delta.position.x =  0.2
        fr_delta.position.y = -0.2
        fr_delta.position.z =  0.0

        robot.go_to('fr_wheel', fr_delta, 4.0, True)
        robot.wait_for_result('fr_wheel')
        
        fr_delta = geomsg.Pose()
        fr_delta.position.x = -0.2
        fr_delta.position.y = 0.2
        fr_delta.position.z = -0.05

        robot.go_to('fr_wheel', fr_delta, 2.0, True)
        robot.wait_for_result('fr_wheel')
        
        return 'done'
        

class Home(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=[])

    def execute(self, userdata):
        
        rospy.loginfo('Executing state HOMING')
        
        robot.load_controller('WheeledMotion')
        
        # CALL ACTION TO GO DOWN AND WAIT FOR RESULT
        (fl, fr, rr, rl, waist) = poses.get_homing()
        
        robot.go_to('fl_wheel', fl, 5.0)
        robot.go_to('fr_wheel', fr, 5.0)
        robot.go_to('hr_wheel', rr, 5.0)
        robot.go_to('hl_wheel', rl, 5.0)
        robot.go_to('waist', waist, 5.0)
        
        robot.wait_for_result('waist')
        robot.wait_for_result('fl_wheel')
        robot.wait_for_result('fr_wheel')
        robot.wait_for_result('hr_wheel')
        robot.wait_for_result('hl_wheel')

        

def main():
    rospy.init_node('push_the_brick_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('GET_STABLE', GetStable(),
                               transitions={'done':'PUSH_BRICK'})
                               
        smach.StateMachine.add('PUSH_BRICK', PushBrick(),
                               transitions={'done':'HOME'})
        
        smach.StateMachine.add('HOME', Home())

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()