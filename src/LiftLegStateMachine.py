#!/usr/bin/env python


import rospy
import smach
import smach_ros
import CentauroCartesianControl
import CommandListener
import LiftLegPoses as poses
import geometry_msgs.msg as geomsg

robot = CentauroCartesianControl.CentauroControl()
# Initial state
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['lift_fr', 'home', 'invalid_input'],
                             output_keys=['wheel_to_lift'])

    def execute(self, userdata):
        rospy.loginfo('Entering IDLE mode')

        cmd_listener = CommandListener.CommandListener("LiftLeg")

        user_command = cmd_listener.get_last_command()
        print "Received " + user_command

        cmd_listener.__del__()

        #user_command = raw_input("Type next state: ")
        if user_command == 'lift_fr':
            userdata.wheel_to_lift = 'FR'
            return 'lift_fr'
        if user_command == 'lift_fl':
            userdata.wheel_to_lift = 'FL'
            return 'lift_fr'
        if user_command == 'lift_rr':
            userdata.wheel_to_lift = 'RR'
            return 'lift_fr'
        if user_command == 'lift_rl':
            userdata.wheel_to_lift = 'RL'
            return 'lift_fr'
        if user_command == 'home':
            userdata.wheel_to_lift = ''
            return 'home'
        else:
            print "Invalid input..try again"
            return 'invalid_input'


class Home(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state HOME')

        # CALL ACTIONS TO SHAPE SUPPORT POLYGON
        (fl, fr, rr, rl) = poses.get_reshape_poly_home()

        robot.go_to('fl_wheel', fl, 0.5)
        robot.go_to('hr_wheel', rr, 0.5)
        robot.go_to('fr_wheel', fr, 0.5)
        robot.go_to('hl_wheel', rl, 0.5)

        robot.wait_for_result('fl_wheel')
        robot.wait_for_result('hr_wheel')
        robot.wait_for_result('fr_wheel')
        robot.wait_for_result('hl_wheel')

        return 'done'

class ShapePolygon(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done'],
                             input_keys=['wheel_to_lift'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RESHAPE POLYGON')
        print 'Received request to lift foot ', userdata.wheel_to_lift
        # CALL ACTIONS TO SHAPE SUPPORT POLYGON
        if userdata.wheel_to_lift == 'FR':
            (fl, fr, rr, rl) = poses.get_reshape_poly_fr()
        if userdata.wheel_to_lift == 'FL':
            (fl, fr, rr, rl) = poses.get_reshape_poly_fl()
        if userdata.wheel_to_lift == 'RR':
            (fl, fr, rr, rl) = poses.get_reshape_poly_rr()
        if userdata.wheel_to_lift == 'RL':
            (fl, fr, rr, rl) = poses.get_reshape_poly_rl()



        robot.go_to('fl_wheel', fl, 0.5)
        robot.go_to('hr_wheel', rr, 0.5)
        robot.go_to('fr_wheel', fr, 0.5)
        robot.go_to('hl_wheel', rl, 0.5)

        robot.wait_for_result('fl_wheel')
        robot.wait_for_result('hr_wheel')
        robot.wait_for_result('fr_wheel')
        robot.wait_for_result('hl_wheel')

        rospy.sleep(0.5)
        return 'done'

class LiftFoot(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done'],
                             input_keys=['wheel_to_lift'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LIFT FOOT')

        goal = geomsg.Pose()
        goal.position.z = 0.25

        if userdata.wheel_to_lift == 'FR':
            robot.go_to('fr_wheel_z', goal, 1.0, True)
            robot.wait_for_result('fr_wheel_z')
        if userdata.wheel_to_lift == 'FL':
            robot.go_to('fl_wheel_z', goal, 1.0, True)
            robot.wait_for_result('fl_wheel_z')
        if userdata.wheel_to_lift == 'RR':
            robot.go_to('hr_wheel_z', goal, 1.0, True)
            robot.wait_for_result('hr_wheel_z')
        if userdata.wheel_to_lift == 'RL':
            robot.go_to('hl_wheel_z', goal, 1.0, True)
            robot.wait_for_result('hl_wheel_z')

        rospy.sleep(0.5)
        return 'done'

class PlaceFoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['wheel_to_lift'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PLACE FOOT')
        print 'Received request to lift foot ', userdata.wheel_to_lift

        goal = geomsg.Pose()
        goal.position.z = -0.25

        if userdata.wheel_to_lift == 'FR':
            robot.go_to('fr_wheel_z', goal, 1.0, True)
            robot.wait_for_result('fr_wheel_z')
        if userdata.wheel_to_lift == 'FL':
            robot.go_to('fl_wheel_z', goal, 1.0, True)
            robot.wait_for_result('fl_wheel_z')
        if userdata.wheel_to_lift == 'RR':
            robot.go_to('hr_wheel_z', goal, 1.0, True)
            robot.wait_for_result('hr_wheel_z')
        if userdata.wheel_to_lift == 'RL':
            robot.go_to('hl_wheel_z', goal, 1.0, True)
            robot.wait_for_result('hl_wheel_z')

        return 'done'


def main():
    rospy.init_node('lift_foot_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])

    # Open the container
    with sm_top:

        smach.StateMachine.add('IDLE', Idle(),
                               transitions={'lift_fr':'RESHAPE_POLY',
                                            'invalid_input':'IDLE',
                                            'home':'HOME'})

        smach.StateMachine.add('HOME', Home(),
                               transitions={'done':'IDLE'})

        smach.StateMachine.add('RESHAPE_POLY', ShapePolygon(),
                               transitions={'done':'LIFT_FOOT'})

        smach.StateMachine.add('LIFT_FOOT', LiftFoot(),
                               transitions={'done':'PLACE_FOOT'})

        smach.StateMachine.add('PLACE_FOOT', PlaceFoot(),
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
