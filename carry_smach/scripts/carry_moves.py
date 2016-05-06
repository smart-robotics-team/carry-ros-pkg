#!/usr/bin/env python

""" carry_moves.py - Version 0.1 2015-02-28

    CARRY state machine

    Created for the CARRY project
    Copyright (c) 2015 Joffrey Kriegel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
import actionlib
import math
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from std_msgs.msg import Int32, Empty, String
from random import randrange
from tf.transformations import quaternion_from_euler
from math import  pi
from collections import OrderedDict
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback



recharge_position = Pose(Point(0.0, 0.0, 0.0), Quaternion(*quaternion_from_euler(0, 0, 0.0, axes='sxyz')))
main_room_position = Pose(Point(0.0, 1.0, 0.0), Quaternion(*quaternion_from_euler(0, 0, 0.0, axes='sxyz')))
corridor_position = Pose(Point(0.0, 2.0, 0.0), Quaternion(*quaternion_from_euler(0, 0, 0.0, axes='sxyz')))
kitchen_position = Pose(Point(0.0, 3.0, 0.0), Quaternion(*quaternion_from_euler(0, 0, 0.0, axes='sxyz')))
bed_room_position = Pose(Point(0.0, 4.0, 0.0), Quaternion(*quaternion_from_euler(0, 0, 0.0, axes='sxyz')))




class Nav2Waypoint(State):
    def __init__(self, goal):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                       input_keys=['waypoint_in'], output_keys=['waypoint_out'])

        # Subscribe to the move_base action server
        #self.move_base = actionlib.SimpleActionClient("CARRY_pathplanner", MoveBaseAction)

        # Wait up to 60 seconds for the action server to become available
        #self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move_base action server")

	self.my_goal = goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'world'

    def execute(self, userdata):
        self.goal.target_pose.pose = self.my_goal 
        self.goal.target_pose.header.frame_id = 'world'

        rospy.loginfo("waypoint_in: " + str(self.goal.target_pose))

        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(self.goal)

        i = 0
        finished_within_time = 0
        while i < 60 and not finished_within_time:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            # Allow 1 minute to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(1))
            #rospy.loginfo('finished ' + str(finished_within_time))

        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return 'aborted'
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                #rospy.sleep(1)   
            return 'succeeded'

class CheckDestination(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted','go_recharge','go_main_room','go_corridor','go_kitchen','go_bed_room'],
                       input_keys=['waypoint_in'], output_keys=['waypoint_out'])

    def execute(self, userdata):

        global recharge_position
        global main_room_position
        global corridor_position
        global kitchen_position
        global bed_room_position

        userdata.waypoint_out = userdata.waypoint_in

        if math.fabs(userdata.waypoint_in.position.x-recharge_position.position.x) < 0.01 and math.fabs(userdata.waypoint_in.position.y-recharge_position.position.y) < 0.01 :
            return 'go_recharge'
        if math.fabs(userdata.waypoint_in.position.x-main_room_position.position.x) < 0.01 and math.fabs(userdata.waypoint_in.position.y-main_room_position.position.y) < 0.01 :
            return 'go_main_room'
        if math.fabs(userdata.waypoint_in.position.x-corridor_position.position.x) < 0.01 and math.fabs(userdata.waypoint_in.position.y-corridor_position.position.y) < 0.01 :
            return 'go_corridor'
        if math.fabs(userdata.waypoint_in.position.x-kitchen_position.position.x) < 0.01 and math.fabs(userdata.waypoint_in.position.y-kitchen_position.position.y) < 0.01 :
            return 'go_kitchen'
        if math.fabs(userdata.waypoint_in.position.x-bed_room_position.position.x) < 0.01 and math.fabs(userdata.waypoint_in.position.y-bed_room_position.position.y) < 0.01 :
            return 'go_bed_room'
        return 'succeeded'



class MoveBack(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','preempted'])

        self.cmd_vel_pub = rospy.Publisher('/CARRY/cmd_vel', Twist, queue_size=2)
        pass

    def execute(self, userdata):
        tmp_twist = Twist()
        tmp_twist.linear.x = -0.04
        self.cmd_vel_pub.publish(tmp_twist)
        rospy.loginfo("Moving back")
        rospy.sleep(0.5)
        return 'succeeded'

class MoveFront(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','preempted'])

        self.cmd_vel_pub = rospy.Publisher('/CARRY/cmd_vel', Twist, queue_size=2)
        pass

    def execute(self, userdata):
        tmp_twist = Twist()
        tmp_twist.linear.x = 0.03
        self.cmd_vel_pub.publish(tmp_twist)
        rospy.loginfo("Moving front")
        return 'succeeded'



class Stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','preempted'])

	self.cmd_vel_pub = rospy.Publisher('/CARRY/cmd_vel', Twist, queue_size=2)
        pass

    def execute(self, userdata):
	self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("Stopping the robot")
        return 'succeeded'

class Pause(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','preempted'])
        pass

    def execute(self, userdata):
        rospy.loginfo("Thinking...")
        rospy.sleep(2.0)   
        return 'succeeded'

class Charging(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','preempted'])

	self.charge_pub = rospy.Publisher('/CARRY/recharge_battery', Empty, queue_size=10)
        pass

    def execute(self, userdata):
        self.charge_pub.publish(Empty())
        rospy.loginfo("Charging... Sleep 2 hours")
        rospy.sleep(2.1*60.0*60.0)
        return 'succeeded'



class Pause2(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             input_keys=['waypoint_in'], output_keys=['waypoint_out'])
        pass

    def execute(self, userdata):
        #rospy.loginfo("Thinking...")
        rospy.loginfo("waypoint_in: " + str(userdata.waypoint_in))
        tmp_data = userdata.waypoint_in
        tmp_data.position.x = tmp_data.position.x + 1.0
        userdata.waypoint_out = tmp_data
        rospy.sleep(2.0)
        return 'succeeded'



class SMACHAI():
    def __init__(self):
        rospy.init_node('carry_smach_move', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        global recharge_position
        global main_room_position
        global corridor_position
        global kitchen_position
        global bed_room_position

        self.recharge_position = recharge_position
        self.main_room_position = main_room_position
        self.corridor_position = corridor_position
        self.kitchen_position = kitchen_position
        self.bed_room_position = bed_room_position


            #StateMachine.add('NAV_S2K_END', Nav2Waypoint(self.waypoints[6]),
            #                 transitions={'succeeded':'IN_KITCHEN',
            #                              'aborted':'aborted'})



	#!!!!!!!!!!!!!!!!!!!!!!!!!!!
	# NEW SM !!!!!!!!!!!!!!!!!!!
	#!!!!!!!!!!!!!!!!!!!!!!!!!!!



        ##################
        # IN CHARGE ROOM 
        ##################

	# Concurrent State machine 
        self.sm_in_charge_room_wait_input = Concurrence(outcomes=['succeeded','aborted','preempted','go_recharge','go_main_room'],
                                        output_keys=['sm_output'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.in_charge_room_wait_input_child_termination_cb,
                                        outcome_cb=self.in_charge_room_wait_input_outcome_cb)

        with self.sm_in_charge_room_wait_input:
            Concurrence.add('GO_TO_KITCHEN', MonitorState("/CARRY/go_kitchen", Empty, self.empty_cb))
            Concurrence.add('GO_TO_BEDROOM', MonitorState("/CARRY/go_bedroom", Empty, self.empty_cb))
            Concurrence.add('GO_TO_MAIN_ROOM', MonitorState("/CARRY/go_main_room", Empty, self.empty_cb))
            Concurrence.add('GO_TO_CORRIDOR', MonitorState("/CARRY/go_corridor", Empty, self.empty_cb))
            Concurrence.add('GO_TO_RECHARGE', MonitorState("/CARRY/go_recharge", Empty, self.empty_cb))

	# Create state machine
        self.sm_in_charge_room = StateMachine(outcomes=['succeeded', 'go_main_room', 'preempted'],
                                              input_keys=['sm_input'],
                                              output_keys=['sm_output'])

        # Add 
        with self.sm_in_charge_room:
            StateMachine.add('WAIT_INPUT', self.sm_in_charge_room_wait_input, transitions={'succeeded':'succeeded',
                                                                                'aborted':'succeeded',
                                                                                'go_recharge':'GO_BACK',
                                                                                'go_main_room':'go_main_room'},
                                                                              remapping={'sm_output':'sm_output'})
            StateMachine.add('TEST', Pause2(),
                             transitions={'succeeded':'go_main_room',
                                          'aborted':'succeeded'},
                             remapping={'waypoint_in':'sm_input',
                                        'waypoint_out':'sm_output'})

            StateMachine.add('GO_BACK', MoveBack(), transitions={'succeeded':'PAUSE_BACK'})
            StateMachine.add('PAUSE_BACK', Pause(), transitions={'succeeded':'STOP_BACK'})
            StateMachine.add('STOP_BACK', Stop(), transitions={'succeeded':'PAUSE_STOP'})
            StateMachine.add('PAUSE_STOP', Pause(), transitions={'succeeded':'CHARGING'})
            StateMachine.add('CHARGING', Charging(), transitions={'succeeded':'GO_FRONT'})
            StateMachine.add('GO_FRONT', MoveFront(), transitions={'succeeded':'PAUSE_FRONT'})
            StateMachine.add('PAUSE_FRONT', Pause(), transitions={'succeeded':'STOP_FRONT'})
            StateMachine.add('STOP_FRONT', Stop(), transitions={'succeeded':'WAIT_INPUT'})



        ##################
        # IN  MAIN  ROOM 
        ##################

        # Concurrent State machine 
        self.sm_in_main_room_wait_input = Concurrence(outcomes=['succeeded','aborted','preempted','go_recharge','go_corridor'],
                                        output_keys=['sm_output'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.in_main_room_wait_input_child_termination_cb,
                                        outcome_cb=self.in_main_room_wait_input_outcome_cb)

        with self.sm_in_main_room_wait_input:
            Concurrence.add('GO_TO_KITCHEN', MonitorState("/CARRY/go_kitchen", Empty, self.empty_cb))
            Concurrence.add('GO_TO_BEDROOM', MonitorState("/CARRY/go_bedroom", Empty, self.empty_cb))
            Concurrence.add('GO_TO_MAIN_ROOM', MonitorState("/CARRY/go_main_room", Empty, self.empty_cb))
            Concurrence.add('GO_TO_CORRIDOR', MonitorState("/CARRY/go_corridor", Empty, self.empty_cb))
            Concurrence.add('GO_TO_RECHARGE', MonitorState("/CARRY/go_recharge", Empty, self.empty_cb))


        # Create state machine
        self.sm_in_main_room = StateMachine(outcomes=['succeeded', 'go_charge_room', 'go_corridor', 'preempted'],
                                            input_keys=['sm_input'],
                                            output_keys=['sm_output'])

        # Add 
        with self.sm_in_main_room:
            StateMachine.add('CHECK_DESTINATION', CheckDestination(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'succeeded',
                                          'go_recharge':'go_charge_room',
                                          'go_main_room':'WAIT_INPUT',
                                          'go_corridor':'go_corridor',
                                          'go_kitchen':'go_corridor',
                                          'go_bed_room':'go_corridor'},
                             remapping={'waypoint_in':'sm_input',
                                        'waypoint_out':'sm_output'})
            StateMachine.add('TEST', Pause2(),
                             transitions={'succeeded':'go_corridor',
                                          'aborted':'succeeded'},
                             remapping={'waypoint_in':'sm_input',
                                        'waypoint_out':'sm_output'})
            StateMachine.add('WAIT_INPUT', self.sm_in_main_room_wait_input, transitions={'succeeded':'WAIT_INPUT',
                                                                                'aborted':'WAIT_INPUT',
                                                                                'go_recharge':'go_charge_room',
                                                                                'go_corridor':'go_corridor'},
                                                                              remapping={'sm_output':'sm_output'})

        ##################
        # IN  CORRIDOR 
        ##################

        # Concurrent State machine 
        self.sm_in_corridor_wait_input = Concurrence(outcomes=['succeeded','aborted','preempted','go_main_room','go_kitchen','go_bed_room'],
                                        output_keys=['sm_output'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.in_corridor_wait_input_child_termination_cb,
                                        outcome_cb=self.in_corridor_wait_input_outcome_cb)

        with self.sm_in_corridor_wait_input:
            Concurrence.add('GO_TO_KITCHEN', MonitorState("/CARRY/go_kitchen", Empty, self.empty_cb))
            Concurrence.add('GO_TO_BEDROOM', MonitorState("/CARRY/go_bedroom", Empty, self.empty_cb))
            Concurrence.add('GO_TO_MAIN_ROOM', MonitorState("/CARRY/go_main_room", Empty, self.empty_cb))
            Concurrence.add('GO_TO_CORRIDOR', MonitorState("/CARRY/go_corridor", Empty, self.empty_cb))
            Concurrence.add('GO_TO_RECHARGE', MonitorState("/CARRY/go_recharge", Empty, self.empty_cb))


        # Create state machine
        self.sm_in_corridor = StateMachine(outcomes=['succeeded', 'go_main_room', 'go_kitchen', 'go_bed_room', 'preempted'],
                                           input_keys=['sm_input'],
                                           output_keys=['sm_output'])

        # Add 
        with self.sm_in_corridor:
            StateMachine.add('CHECK_DESTINATION', CheckDestination(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'succeeded',
                                          'go_recharge':'go_main_room',
                                          'go_main_room':'go_main_room',
                                          'go_corridor':'WAIT_INPUT',
                                          'go_kitchen':'go_kitchen',
                                          'go_bed_room':'go_bed_room'},
                             remapping={'waypoint_in':'sm_input',
                                        'waypoint_out':'sm_output'})
            StateMachine.add('TEST', Pause2(),
                             transitions={'succeeded':'go_kitchen',
                                          'aborted':'succeeded'},
                             remapping={'waypoint_in':'sm_input',
                                        'waypoint_out':'sm_output'})
            StateMachine.add('WAIT_INPUT', self.sm_in_corridor_wait_input, transitions={'succeeded':'WAIT_INPUT',
                                                                                'aborted':'WAIT_INPUT',
                                                                                'go_main_room':'go_main_room',
                                                                                'go_kitchen':'go_kitchen',
                                                                                'go_bed_room':'go_bed_room'},
                                                                              remapping={'sm_output':'sm_output'})

        ##################
        # IN  KITCHEN 
        ##################


        # Concurrent State machine 
        self.sm_in_kitchen_wait_input = Concurrence(outcomes=['succeeded','aborted','preempted','go_corridor'],
                                        output_keys=['sm_output'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.in_kitchen_wait_input_child_termination_cb,
                                        outcome_cb=self.in_kitchen_wait_input_outcome_cb)

        with self.sm_in_kitchen_wait_input:
            Concurrence.add('GO_TO_KITCHEN', MonitorState("/CARRY/go_kitchen", Empty, self.empty_cb))
            Concurrence.add('GO_TO_BEDROOM', MonitorState("/CARRY/go_bedroom", Empty, self.empty_cb))
            Concurrence.add('GO_TO_MAIN_ROOM', MonitorState("/CARRY/go_main_room", Empty, self.empty_cb))
            Concurrence.add('GO_TO_CORRIDOR', MonitorState("/CARRY/go_corridor", Empty, self.empty_cb))
            Concurrence.add('GO_TO_RECHARGE', MonitorState("/CARRY/go_recharge", Empty, self.empty_cb))


        # Create state machine
        self.sm_in_kitchen = StateMachine(outcomes=['succeeded', 'go_corridor', 'preempted'],
                                          input_keys=['sm_input'],
                                          output_keys=['sm_output'])

        # Add 
        with self.sm_in_kitchen:
            StateMachine.add('WAIT_INPUT', self.sm_in_kitchen_wait_input, transitions={'succeeded':'WAIT_INPUT',
                                                                                'aborted':'WAIT_INPUT',
                                                                                'go_corridor':'go_corridor'},
                                                                              remapping={'sm_output':'sm_output'})
            StateMachine.add('TEST', Pause2(),
                             transitions={'succeeded':'go_corridor',
                                          'aborted':'succeeded'},
                             remapping={'waypoint_in':'sm_input',
                                        'waypoint_out':'sm_output'})
            StateMachine.add('CHECK_DESTINATION', CheckDestination(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'succeeded',
                                          'go_recharge':'go_corridor',
                                          'go_main_room':'go_corridor',
                                          'go_corridor':'go_corridor',
                                          'go_kitchen':'WAIT_INPUT',
                                          'go_bed_room':'go_corridor'},
                             remapping={'waypoint_in':'sm_input',
                                        'waypoint_out':'sm_output'})


        ##################
        # IN  BED  ROOM 
        ##################


        # Concurrent State machine 
        self.sm_in_bed_room_wait_input = Concurrence(outcomes=['succeeded','aborted','preempted','go_corridor'],
                                        output_keys=['sm_output'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.in_bed_room_wait_input_child_termination_cb,
                                        outcome_cb=self.in_bed_room_wait_input_outcome_cb)

        with self.sm_in_bed_room_wait_input:
            Concurrence.add('GO_TO_KITCHEN', MonitorState("/CARRY/go_kitchen", Empty, self.empty_cb))
            Concurrence.add('GO_TO_BEDROOM', MonitorState("/CARRY/go_bedroom", Empty, self.empty_cb))
            Concurrence.add('GO_TO_MAIN_ROOM', MonitorState("/CARRY/go_main_room", Empty, self.empty_cb))
            Concurrence.add('GO_TO_CORRIDOR', MonitorState("/CARRY/go_corridor", Empty, self.empty_cb))
            Concurrence.add('GO_TO_RECHARGE', MonitorState("/CARRY/go_recharge", Empty, self.empty_cb))


        # Create state machine
        self.sm_in_bed_room = StateMachine(outcomes=['succeeded', 'go_corridor', 'preempted'],
                                           input_keys=['sm_input'],
                                           output_keys=['sm_output'])

        # Add 
        with self.sm_in_bed_room:
            StateMachine.add('WAIT_INPUT', self.sm_in_bed_room_wait_input, transitions={'succeeded':'WAIT_INPUT',
                                                                                'aborted':'WAIT_INPUT',
                                                                                'go_corridor':'go_corridor'},
                                                                              remapping={'sm_output':'sm_output'})
            StateMachine.add('TEST', Pause2(),
                             transitions={'succeeded':'go_corridor',
                                          'aborted':'succeeded'},
                             remapping={'waypoint_in':'sm_input',
                                        'waypoint_out':'sm_output'})
            StateMachine.add('CHECK_DESTINATION', CheckDestination(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'succeeded',
                                          'go_recharge':'go_corridor',
                                          'go_main_room':'go_corridor',
                                          'go_corridor':'go_corridor',
                                          'go_kitchen':'go_corridor',
                                          'go_bed_room':'WAIT_INPUT'},
                             remapping={'waypoint_in':'sm_input',
                                        'waypoint_out':'sm_output'})



        ##################
        # TOP SM 
        ##################

	# Create the top level state machine
        self.sm_top = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
	self.sm_top.userdata.goal = self.recharge_position

        # Add nav_patrol, sm_recharge and a Stop() machine to sm_top
        with self.sm_top:
            StateMachine.add('IN_CHARGE_ROOM', self.sm_in_charge_room, transitions={'succeeded':'IN_CHARGE_ROOM',
                                                                                'go_main_room':'IN_MAIN_ROOM'},
                                                                       remapping={'sm_input':'goal', 'sm_output':'goal'})

            StateMachine.add('IN_MAIN_ROOM', self.sm_in_main_room, transitions={'succeeded':'IN_MAIN_ROOM',
                                                                                'go_charge_room':'IN_CHARGE_ROOM',
                                                                                'go_corridor':'IN_CORRIDOR'},
                                                                   remapping={'sm_input':'goal', 'sm_output':'goal'})

            StateMachine.add('IN_CORRIDOR', self.sm_in_corridor, transitions={'succeeded':'IN_CORRIDOR',
                                                                                'go_main_room':'IN_MAIN_ROOM',
                                                                                'go_kitchen':'IN_KITCHEN',
                                                                                'go_bed_room':'IN_BED_ROOM'},
                                                                 remapping={'sm_input':'goal', 'sm_output':'goal'})

            StateMachine.add('IN_KITCHEN', self.sm_in_kitchen, transitions={'succeeded':'IN_KITCHEN',
                                                                                'go_corridor':'IN_CORRIDOR'},
                                                               remapping={'sm_input':'goal', 'sm_output':'goal'})

            StateMachine.add('IN_BED_ROOM', self.sm_in_bed_room, transitions={'succeeded':'IN_BED_ROOM',
                                                                                'go_corridor':'IN_CORRIDOR'},
                                                                 remapping={'sm_input':'goal', 'sm_output':'goal'})






        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('carry_sm', self.sm_top, '/SM_CARRY_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = self.sm_top.execute()
        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
                
        intro_server.stop()



    def empty_cb(self, userdata, msg):
	#rospy.loginfo("Empty message received.")
        return False


    # Gets called when ANY child state terminates
    def in_charge_room_wait_input_child_termination_cb(self, outcome_map):
        #rospy.loginfo("daymode_child_termination_cb.")
        return True


    # Gets called when ALL child states are terminated
    def in_charge_room_wait_input_outcome_cb(self, outcome_map):
        #rospy.loginfo("daymode_outcome_cb.")
        if outcome_map['GO_TO_RECHARGE'] == 'invalid':
            self.sm_in_charge_room_wait_input.userdata.sm_output = self.recharge_position
            self.sm_in_charge_room.userdata.sm_input = self.recharge_position
            rospy.loginfo("Going to charge ")
            return 'go_recharge'
        if outcome_map['GO_TO_MAIN_ROOM'] == 'invalid':
            self.sm_in_charge_room_wait_input.userdata.sm_output = self.main_room_position
            self.sm_in_charge_room.userdata.sm_input = self.main_room_position
            rospy.loginfo("Going to the main room")
            return 'go_main_room'
        if outcome_map['GO_TO_CORRIDOR'] == 'invalid':
            self.sm_in_charge_room_wait_input.userdata.sm_output = self.corridor_position
            self.sm_in_charge_room.userdata.sm_input = self.corridor_position
            rospy.loginfo("Going to the corridor")
            return 'go_main_room'
        if outcome_map['GO_TO_KITCHEN'] == 'invalid':
            self.sm_in_charge_room_wait_input.userdata.sm_output = self.kitchen_position
            self.sm_in_charge_room.userdata.sm_input = self.kitchen_position
            rospy.loginfo("Going to the kitchen")
            return 'go_main_room'
        if outcome_map['GO_TO_BEDROOM'] == 'invalid':
            self.sm_in_charge_room_wait_input.userdata.sm_output = self.bed_room_position
            self.sm_in_charge_room.userdata.sm_input = self.bed_room_position
            rospy.loginfo("Going to the bed room")
            return 'go_main_room'
        else:
            return 'aborted'

    # Gets called when ANY child state terminates
    def in_main_room_wait_input_child_termination_cb(self, outcome_map):
        #rospy.loginfo("daymode_child_termination_cb.")
        return True


    # Gets called when ALL child states are terminated
    def in_main_room_wait_input_outcome_cb(self, outcome_map):
        #rospy.loginfo("daymode_outcome_cb.")
        if outcome_map['GO_TO_RECHARGE'] == 'invalid':
            self.sm_in_main_room_wait_input.userdata.sm_output = self.recharge_position
            self.sm_in_main_room.userdata.sm_input = self.recharge_position
            rospy.loginfo("Going to charge ")
            return 'go_recharge'
        if outcome_map['GO_TO_MAIN_ROOM'] == 'invalid':
            self.sm_in_main_room_wait_input.userdata.sm_output = self.main_room_position
            self.sm_in_main_room.userdata.sm_input = self.main_room_position
            rospy.loginfo("Going to the main room")
            return 'succeeded'
        if outcome_map['GO_TO_CORRIDOR'] == 'invalid':
            self.sm_in_main_room_wait_input.userdata.sm_output = self.corridor_position
            self.sm_in_main_room.userdata.sm_input = self.corridor_position
            rospy.loginfo("Going to the corridor")
            return 'go_corridor'
        if outcome_map['GO_TO_KITCHEN'] == 'invalid':
            self.sm_in_main_room_wait_input.userdata.sm_output = self.kitchen_position
            self.sm_in_main_room.userdata.sm_input = self.kitchen_position
            rospy.loginfo("Going to the kitchen")
            return 'go_corridor'
        if outcome_map['GO_TO_BEDROOM'] == 'invalid':
            self.sm_in_main_room_wait_input.userdata.sm_output = self.bed_room_position
            self.sm_in_main_room.userdata.sm_input = self.bed_room_position
            rospy.loginfo("Going to the bed room")
            return 'go_corridor'
        else:
            return 'aborted'

    # Gets called when ANY child state terminates
    def in_corridor_wait_input_child_termination_cb(self, outcome_map):
        #rospy.loginfo("daymode_child_termination_cb.")
        return True


    # Gets called when ALL child states are terminated
    def in_corridor_wait_input_outcome_cb(self, outcome_map):
        #rospy.loginfo("daymode_outcome_cb.")
        if outcome_map['GO_TO_RECHARGE'] == 'invalid':
            self.sm_in_corridor_wait_input.userdata.sm_output = self.recharge_position
            self.sm_in_corridor.userdata.sm_input = self.recharge_position
            rospy.loginfo("Going to charge ")
            return 'go_main_room'
        if outcome_map['GO_TO_MAIN_ROOM'] == 'invalid':
            self.sm_in_corridor_wait_input.userdata.sm_output = self.main_room_position
            self.sm_in_corridor.userdata.sm_input = self.main_room_position
            rospy.loginfo("Going to the main room")
            return 'go_main_room'
        if outcome_map['GO_TO_CORRIDOR'] == 'invalid':
            self.sm_in_corridor_wait_input.userdata.sm_output = self.corridor_position
            self.sm_in_corridor.userdata.sm_input = self.corridor_position
            rospy.loginfo("Going to the corridor")
            return 'succeeded'
        if outcome_map['GO_TO_KITCHEN'] == 'invalid':
            self.sm_in_corridor_wait_input.userdata.sm_output = self.kitchen_position
            self.sm_in_corridor.userdata.sm_input = self.kitchen_position
            rospy.loginfo("Going to the kitchen")
            return 'go_kitchen'
        if outcome_map['GO_TO_BEDROOM'] == 'invalid':
            self.sm_in_corridor_wait_input.userdata.sm_output = self.bed_room_position
            self.sm_in_corridor.userdata.sm_input = self.bed_room_position
            rospy.loginfo("Going to the bed room")
            return 'go_bed_room'
        else:
            return 'aborted'

    # Gets called when ANY child state terminates
    def in_kitchen_wait_input_child_termination_cb(self, outcome_map):
        #rospy.loginfo("daymode_child_termination_cb.")
        return True


    # Gets called when ALL child states are terminated
    def in_kitchen_wait_input_outcome_cb(self, outcome_map):
        #rospy.loginfo("daymode_outcome_cb.")
        if outcome_map['GO_TO_RECHARGE'] == 'invalid':
            self.sm_in_kitchen_wait_input.userdata.sm_output = self.recharge_position
            self.sm_in_kitchen.userdata.sm_input = self.recharge_position
            rospy.loginfo("Going to charge ")
            return 'go_corridor'
        if outcome_map['GO_TO_MAIN_ROOM'] == 'invalid':
            self.sm_in_kitchen_wait_input.userdata.sm_output = self.main_room_position
            self.sm_in_kitchen.userdata.sm_input = self.main_room_position
            rospy.loginfo("Going to the main room")
            return 'go_corridor'
        if outcome_map['GO_TO_CORRIDOR'] == 'invalid':
            self.sm_in_kitchen_wait_input.userdata.sm_output = self.corridor_position
            self.sm_in_kitchen.userdata.sm_input = self.corridor_position
            rospy.loginfo("Going to the corridor")
            return 'go_corridor'
        if outcome_map['GO_TO_KITCHEN'] == 'invalid':
            self.sm_in_kitchen_wait_input.userdata.sm_output = self.kitchen_position
            self.sm_in_kitchen.userdata.sm_input = self.kitchen_position
            rospy.loginfo("Going to the kitchen")
            return 'succeeded'
        if outcome_map['GO_TO_BEDROOM'] == 'invalid':
            self.sm_in_kitchen_wait_input.userdata.sm_output = self.bed_room_position
            self.sm_in_kitchen.userdata.sm_input = self.bed_room_position
            rospy.loginfo("Going to the bed room")
            return 'go_corridor'
        else:
            return 'aborted'

    # Gets called when ANY child state terminates
    def in_bed_room_wait_input_child_termination_cb(self, outcome_map):
        #rospy.loginfo("daymode_child_termination_cb.")
        return True


    # Gets called when ALL child states are terminated
    def in_bed_room_wait_input_outcome_cb(self, outcome_map):
        #rospy.loginfo("daymode_outcome_cb.")
        if outcome_map['GO_TO_RECHARGE'] == 'invalid':
            self.sm_in_bed_room_wait_input.userdata.sm_output = self.recharge_position
            self.sm_in_bed_room.userdata.sm_input = self.recharge_position
            rospy.loginfo("Going to charge ")
            return 'go_corridor'
        if outcome_map['GO_TO_MAIN_ROOM'] == 'invalid':
            self.sm_in_bed_room_wait_input.userdata.sm_output = self.main_room_position
            self.sm_in_bed_room.userdata.sm_input = self.main_room_position
            rospy.loginfo("Going to the main room")
            return 'go_corridor'
        if outcome_map['GO_TO_CORRIDOR'] == 'invalid':
            self.sm_in_bed_room_wait_input.userdata.sm_output = self.corridor_position
            self.sm_in_bed_room.userdata.sm_input = self.corridor_position
            rospy.loginfo("Going to the corridor")
            return 'go_corridor'
        if outcome_map['GO_TO_KITCHEN'] == 'invalid':
            self.sm_in_bed_room_wait_input.userdata.sm_output = self.kitchen_position
            self.sm_in_bed_room.userdata.sm_input = self.kitchen_position
            rospy.loginfo("Going to the kitchen")
            return 'go_corridor'
        if outcome_map['GO_TO_BEDROOM'] == 'invalid':
            self.sm_in_bed_room_wait_input.userdata.sm_output = self.bed_room_position
            self.sm_in_bed_room.userdata.sm_input = self.bed_room_position
            rospy.loginfo("Going to the bed room")
            return 'succeeded'
        else:
            return 'aborted'


















    def time_cb(self, userdata, msg):
        if msg.data < 2:
            self.stopping = True
            return False
        else:
            self.stopping = False
            return True

	

    def shutdown(self):
        rospy.loginfo("Stopping carry automation...")
        
        #self.sm_day_mode.request_preempt()
        
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        SMACHAI()
    except rospy.ROSInterruptException:
        rospy.loginfo("CARRY automation finished.")
