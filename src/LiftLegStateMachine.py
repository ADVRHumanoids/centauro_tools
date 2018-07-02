#!/usr/bin/env python


import rospy
import smach
import smach_ros
import CentauroCartesianControl 
import LiftLegPoses as poses
import geometry_msgs.msg as geomsg

robot = CentauroCartesianControl.CentauroControl()



# Initial state
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lift_fr', 'invalid_input'])

    def execute(self, userdata):
        rospy.loginfo('Entering IDLE mode')
        user_command = raw_input("Type next state: ") 
        if user_command == 'lift_fr':
            return 'lift_fr'
        else:
            print "Invalid input..try again"
            return 'invalid_input'

class ShapePolygon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RESHAPE POLYGON')
        
        # CALL ACTIONS TO SHAPE SUPPORT POLYGON
        (fl, rr) = poses.get_reshape_poly_03()


        robot.go_to('fl_wheel', fl, 6.0)
        robot.go_to('hr_wheel', rr, 6.0)

        robot.wait_for_result('fl_wheel')
        robot.wait_for_result('hr_wheel')

        return 'done'

class LiftFoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LIFT FOOT')

        return 'done'
        


def main():
    rospy.init_node('lift_foot_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'lift_fr':'RESHAPE_POLY',
                                            'invalid_input':'IDLE'})
                                   

        smach.StateMachine.add('RESHAPE_POLY', ShapePolygon(),
                               transitions={'done':'LIFT_FOOT'})
                               
        smach.StateMachine.add('LIFT_FOOT', LiftFoot(),
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