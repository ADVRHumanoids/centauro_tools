#!/usr/bin/env python


import rospy
import smach
import smach_ros
import CentauroCartesianControl 
import GroundManipulationPoses as poses
import geometry_msgs.msg as geomsg

robot = CentauroCartesianControl.CentauroControl()



# Initial state
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_down','invalid_input'])

    def execute(self, userdata):
        rospy.loginfo('Entering IDLE mode')
        user_command = raw_input("Type next state: ") 
        if user_command == self.get_registered_outcomes()[0]:
            return self.get_registered_outcomes()[0]
        else:
            print "Invalid input"
            return 'invalid_input'

class ClearWs(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ws_cleared'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CLEAR WORKSPACE')
        
        # CHANGE CONTROLLER TO WHEELED MOTION
        robot.load_controller('WheeledMotion')
        
        # CALL ACTION TO GO DOWN AND WAIT FOR RESULT
        (l_pose, r_pose) = poses.get_clear_workspace_lr()
        waist_delta_pose = geomsg.Pose()
        waist_delta_pose.position.z = -0.1

        robot.go_to('waist', waist_delta_pose, 6.0, True)
        robot.go_to('fl_wheel', l_pose, 6.0)
        robot.go_to('fr_wheel', r_pose, 6.0)
        
        robot.wait_for_result('waist')
        robot.wait_for_result('fl_wheel')
        robot.wait_for_result('fr_wheel')
        
        # BACK TO FIXED-WHEELS CONTROLLER
        robot.load_controller('OpenSot')
        
        text = ''
        while not (text == 'Y' or text == 'n'):
            text = raw_input('Workspace cleared correcly? [Y/n]')
            
        if text == 'Y':
            return 'ws_cleared'
        else:
            return 'failure'


class Down(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DOWN')
        
        delta_pose = geomsg.Pose()
        delta_pose.position.x = 0.3
        
        robot.lhand_go_to(delta_pose, 2.0, incremental = True)
        robot.rhand_go_to(delta_pose, 2.0, incremental = True)
        robot.lhand_wait_for_result()
        robot.rhand_wait_for_result()
        
        # CALL ACTION TO GO DOWN AND WAIT FOR RESULT
        (l_pose, r_pose) = poses.get_down_open_lr()

        robot.lhand_go_to(l_pose, 12.0)
        robot.rhand_go_to(r_pose, 12.0)
        robot.lhand_wait_for_result()
        robot.rhand_wait_for_result()
        
        return 'success'
        
        
class Closing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failure', 'success'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CLOSING')
        
        # CALL ACTION TO CLOSE HANDS
#        (l_pose, r_pose) = poses.get_down_close_lr()
#        
#
#        robot.lhand_go_to(l_pose, 2.0)
#        robot.rhand_go_to(r_pose, 2.0)
#        
#        # MONITOR FT INSIDE A LOOP
#        loop_rate = rospy.Rate(50)
#        exit_loop = False
#        closing_complete = False
        ft_threshold_reached = False
        
#        while not exit_loop:
#            closing_complete = robot.rhand_getstate() == 'succeeded' and robot.lhand_getstate() == 'succeeded'
#            l_ft, r_ft = robot.get_arm_ft_lr()
#            rospy.loginfo('FT Z value: left = %f, right = %f', l_ft.force.z, r_ft.force.z)
#            ft_threshold_reached = abs(l_ft.force.z) > 20 and abs(r_ft.force.z) > 20
#            exit_loop = closing_complete or ft_threshold_reached
#            loop_rate.sleep()
        
        if ft_threshold_reached:
            return 'success'
        else:
            return 'failure'


class Up(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'success_no_box'])

    def execute(self, userdata):
        rospy.loginfo('Executing state UP')
        
        # CALL ACTION TO GO UP
        (l_pose, r_pose) = poses.get_up_close_lr()
        robot.lhand_go_to(l_pose, 6.0)
        robot.rhand_go_to(r_pose, 6.0)
        robot.lhand_wait_for_result()
        robot.rhand_wait_for_result()
        
        grasp_success = raw_input("Did we pick the box? [Y/n] ") == 'Y'
        
        if grasp_success:
            return 'success'
        else:
            return 'success_no_box'


class PassBox(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state PASS_BOX')
        
        robot.set_base_link('l_hand', 'DWYTorso')
        robot.set_base_link('r_hand', 'DWYTorso')
        
        pose = poses.get_pass_box_torso()
        
        robot.torso_go_to(pose, 3.0)
        robot.torso_wait_for_result()
        
        raw_input("Press ENTER to release box")
        
        # Release box
        delta_pose = geomsg.Pose()
        delta_pose.position.x = 0.0
        delta_pose.position.y = 0.05
        delta_pose.position.z = 0.0
        delta_pose.orientation.x = 0.0
        delta_pose.orientation.y = 0.0
        delta_pose.orientation.z = 0.0
        delta_pose.orientation.w = 1.0
        robot.lhand_go_to(delta_pose, 2.0, incremental=True)
        
        delta_pose = geomsg.Pose()
        delta_pose.position.x = 0.0
        delta_pose.position.y = -0.05
        delta_pose.position.z = 0.0
        delta_pose.orientation.x = 0.0
        delta_pose.orientation.y = 0.0
        delta_pose.orientation.z = 0.0
        delta_pose.orientation.w = 1.0
        robot.rhand_go_to(delta_pose, 2.0, incremental=True)
        
        robot.rhand_wait_for_result()
        robot.lhand_wait_for_result()
        
        
        return self.get_registered_outcomes()[0]
        
        
class BackHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state BACK_HOME')
        
        # CHANGE CONTROLLER TO WHEELED MOTION
        robot.load_controller('WheeledMotion')
        
        # CALL ACTION TO GO DOWN AND WAIT FOR RESULT
        (l_pose, r_pose) = poses.get_wheels_home_lr()
        

        robot.go_to('fl_wheel', l_pose, 6.0)
        robot.go_to('fr_wheel', r_pose, 6.0)
        
        robot.wait_for_result('fl_wheel')
        robot.wait_for_result('fr_wheel')
        
        # BACK TO FIXED-WHEELS CONTROLLER
        # robot.load_controller('OpenSot')
        
        return self.get_registered_outcomes()[0]
        


def main():
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'go_down':'DOWN_SM',
                                            'invalid_input':'IDLE'})

        # Create the sub SMACH state machine
        sm_down = smach.StateMachine(outcomes=['closed', 'aborted'])

        # Open the container
        with sm_down:

            smach.StateMachine.add('CLEAR_WORKSPACE', ClearWs(), 
                                   transitions={'ws_cleared':'DOWN'})
                                   
            smach.StateMachine.add('DOWN', Down(), 
                                   transitions={'success':'CLOSING'})
                                   
            smach.StateMachine.add('CLOSING', Closing(), 
                                   transitions={'success':'closed', 
                                                'failure':'aborted'})
                                   
                                   

        smach.StateMachine.add('DOWN_SM', sm_down,
                               transitions={'closed':'UP', 
                                            'aborted':'UP'})
                                            
        smach.StateMachine.add('UP', Up(),
                               transitions={'success':'PASS_BOX', 
                                            'success_no_box':'BACK_HOME'})
                               
        smach.StateMachine.add('PASS_BOX', PassBox(),
                               transitions={'success':'BACK_HOME'})
                               
        smach.StateMachine.add('BACK_HOME', BackHome(),
                               transitions={'success':'IDLE'})
                                            
                                            

    
    
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