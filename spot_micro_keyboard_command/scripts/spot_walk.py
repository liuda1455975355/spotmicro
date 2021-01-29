#!/usr/bin/python
import rospy
import sched, time
import sys, select, termios, tty # For terminal keyboard key press reading
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Vector3
from math import pi

msg = """
spot_walk  function descripetion

start monitoring
subscriber the topic from the websocket
control the robot go straight ,stop,  turn left and turn right


CTRL-C to quit
"""
speed_inc = 0.012
yaw_rate_inc = 1.5*pi/180
angle_inc = 2*pi/180

forward_s = """forward"""
stop_s = """stop"""
left_s = """left"""
right_s = """right"""

forward_flag = 0
stop_flag = 0
left_flag = 0
right_flag = 0

class SpotMicroKeyboardControl():
    def __init__(self):

        # Create messages for body motion commands, and initialize to zero 

        self._speed_cmd_msg = Vector3()
        self._speed_cmd_msg.x = 0
        self._speed_cmd_msg.y = 0
        self._speed_cmd_msg.z = 0

        self._walk_event_cmd_msg = Bool()
        self._walk_event_cmd_msg.data = True # Mostly acts as an event driven action on receipt of a true message

        self._stand_event_cmd_msg = Bool()
        self._stand_event_cmd_msg.data = True
       
        self._idle_event_cmd_msg = Bool()
        self._idle_event_cmd_msg.data = True

        rospy.loginfo("Setting Up the Spot Micro Voice Control Node...")

        # Set up and title the ros node for this code
        rospy.init_node('spot_micro_keyboard_control')
        #rospy.Subscriber('voiceWords', String, callbackv)
        #rospy.spin()

        # Create publishers for x,y speed command, body rate command, and state command
        self.ros_pub_speed_cmd      = rospy.Publisher('/speed_cmd',Vector3,queue_size=1)
        self.ros_pub_walk_cmd       = rospy.Publisher('/walk_cmd',Bool, queue_size=1)
        self.ros_pub_stand_cmd      = rospy.Publisher('/stand_cmd',Bool,queue_size=1)
        self.ros_pub_idle_cmd       = rospy.Publisher('/idle_cmd',Bool,queue_size=1)

        #

        rospy.loginfo("> Publishers corrrectly initialized")

        rospy.loginfo("Initialization complete")

        # Setup terminal input reading, taken from teleop_twist_keyboard
        self.settings = termios.tcgetattr(sys.stdin)

    def reset_all_motion_commands_to_zero(self):
        '''Reset body motion cmd states to zero and publish zero value body motion commands'''

        self._speed_cmd_msg.x = 0
        self._speed_cmd_msg.y = 0
        self._speed_cmd_msg.z = 0

        self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

    def run(self):
        
        global forward_flag ,stop_flag,left_flag ,right_flag 
        def callbackv(data):

            global forward_flag ,stop_flag,left_flag ,right_flag 
            rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

            if (data.data == forward_s):
                forward_flag = 1
        
            if (data.data == stop_s):
                stop_flag = 1
        
            if (data.data == left_s):
                left_flag = 1

            if (data.data == right_s):
                right_flag = 1
        
        self.reset_all_motion_commands_to_zero()
        #publish the stand mode
        self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)

        rospy.Subscriber('spotwebsocket', String, callbackv)

        while not rospy.is_shutdown():
            #print(msg)
            rospy.loginfo('wait for the commond ')
            time.sleep(1)
            if (forward_flag == 1):
                self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)
                
                print('walk forward')
                time.sleep(0.1)
                self._speed_cmd_msg.x = 0.05
                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s ,z speed:%1.3f m/s'\
                        %(self._speed_cmd_msg.x,self._speed_cmd_msg.y,self._speed_cmd_msg.z)) 
                
                forward_flag = 0

            elif(stop_flag):
                self.reset_all_motion_commands_to_zero()
                self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                print('stop ')
                stop_flag = 0
            
            elif(left_flag):

                self.reset_all_motion_commands_to_zero()
                self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)
                print('turn left')

                time.sleep( 0.1 )

                self._speed_cmd_msg.z = 20*pi/180
                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s ,z speed:%1.3f m/s'\
                        %(self._speed_cmd_msg.x,self._speed_cmd_msg.y,self._speed_cmd_msg.z)) 

                time.sleep( 5 )

                self.reset_all_motion_commands_to_zero()

                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)
                self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)

                left_flag = 0

            elif(right_flag):

                self.reset_all_motion_commands_to_zero()
                self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)
                print('turn right')

                time.sleep( 0.1 )

                self._speed_cmd_msg.z = -20*pi/180
                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s ,z speed:%1.3f m/s'\
                        %(self._speed_cmd_msg.x,self._speed_cmd_msg.y,self._speed_cmd_msg.z)) 

                time.sleep( 5 )

                self.reset_all_motion_commands_to_zero()

                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)
                self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)

                left_flag = 0     

if __name__ == "__main__":
    smkc     = SpotMicroKeyboardControl()
    smkc.run()
    