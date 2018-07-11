#!/usr/bin/env python


import rospy
import smach
import smach_ros
import CentauroCartesianControl 
import TerrainAdaptationPoses as poses
import geometry_msgs.msg as geomsg
import tf

robot = CentauroCartesianControl.CentauroControl()



# Initial state
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['up_stretch', 'down_mid', 'down_low', 'invalid_input'])

    def execute(self, userdata):
        rospy.loginfo('Entering IDLE mode')
        user_command = raw_input("Type next state: ") 
        if user_command == 'up_stretch':
            return 'up_stretch'
        if user_command == 'down_mid':
            return 'down_mid'
        if user_command == 'down_low':
            return 'down_low'
        else:
            print "Invalid input..try again"
            return 'invalid_input'

class UpStretch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state UP STRETCHED')
        
        # CALL ACTION TO GO DOWN AND WAIT FOR RESULT
        (fl, fr, rr, rl, waist) = poses.get_up_stretched_0123wa()
        
        try:
            (wa_trans, wa_rot) = robot.get_current_pose('waist');
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Unable to lookup TF'
            return 'done'
            
        waist.position.x = wa_trans[0]
        waist.position.y = wa_trans[1]
        waist.orientation.x = wa_rot[0]
        waist.orientation.y = wa_rot[1]
        waist.orientation.z = wa_rot[2]
        waist.orientation.w = wa_rot[3]

        robot.go_to('waist', waist, 6.0)
        robot.go_to('fl_wheel', fl, 6.0)
        robot.go_to('fr_wheel', fr, 6.0)
        robot.go_to('hr_wheel', rr, 6.0)
        robot.go_to('hl_wheel', rl, 6.0)
        
        robot.wait_for_result('waist')
        robot.wait_for_result('fl_wheel')
        robot.wait_for_result('fr_wheel')
        robot.wait_for_result('hr_wheel')
        robot.wait_for_result('hl_wheel')

        
        return 'done'

class DownMid(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state UP STRETCHED')
        
        # CALL ACTION TO GO DOWN AND WAIT FOR RESULT
        (fl, fr, rr, rl, waist) = poses.get_down_mid_0123wa()
        
        try:
            (wa_trans, wa_rot) = robot.get_current_pose('waist');
            print wa_trans, wa_rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Unable to lookup TF'
            return 'done'
            
        waist.position.x = wa_trans[0]
        waist.position.y = wa_trans[1]
        waist.orientation.x = wa_rot[0]
        waist.orientation.y = wa_rot[1]
        waist.orientation.z = wa_rot[2]
        waist.orientation.w = wa_rot[3]

        robot.go_to('waist', waist, 6.0)
        robot.go_to('fl_wheel', fl, 6.0)
        robot.go_to('fr_wheel', fr, 6.0)
        robot.go_to('hr_wheel', rr, 6.0)
        robot.go_to('hl_wheel', rl, 6.0)
        
        robot.wait_for_result('waist')
        robot.wait_for_result('fl_wheel')
        robot.wait_for_result('fr_wheel')
        robot.wait_for_result('hr_wheel')
        robot.wait_for_result('hl_wheel')

        
        return 'done'
        
class DownLow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state UP STRETCHED')
        
        # CALL ACTION TO GO DOWN AND WAIT FOR RESULT
        (fl, fr, rr, rl, waist) = poses.get_down_low_0123wa()
        
        try:
            (wa_trans, wa_rot) = robot.get_current_pose('waist');
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Unable to lookup TF'
            return 'done'
            
        waist.position.x = wa_trans[0]
        waist.position.y = wa_trans[1]
        waist.orientation.x = wa_rot[0]
        waist.orientation.y = wa_rot[1]
        waist.orientation.z = wa_rot[2]
        waist.orientation.w = wa_rot[3]

        robot.go_to('waist', waist, 6.0)
        robot.go_to('fl_wheel', fl, 6.0)
        robot.go_to('fr_wheel', fr, 6.0)
        robot.go_to('hr_wheel', rr, 6.0)
        robot.go_to('hl_wheel', rl, 6.0)
        
        robot.wait_for_result('waist')
        robot.wait_for_result('fl_wheel')
        robot.wait_for_result('fr_wheel')
        robot.wait_for_result('hr_wheel')
        robot.wait_for_result('hl_wheel')

        
        return 'done'


def main():
    rospy.init_node('terrain_adaptation_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'up_stretch':'UP_STRETCH',
                                            'down_mid':'DOWN_MID',
                                            'down_low':'DOWN_LOW',
                                            'invalid_input':'IDLE'})
                                   

        smach.StateMachine.add('UP_STRETCH', UpStretch(),
                               transitions={'done':'IDLE'})
                               
        smach.StateMachine.add('DOWN_MID', DownMid(),
                               transitions={'done':'IDLE'})
                               
        smach.StateMachine.add('DOWN_LOW', DownLow(),
                               transitions={'done':'IDLE'})

                                            
                                            

    
    
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