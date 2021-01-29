#!/usr/bin/python
import rospy
import sched, time
import sys, select, termios, tty # For terminal keyboard key press reading
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Vector3
from math import pi

msg = """
 voice_control  function descripetion

start listening


CTRL-C to quit
"""

speed_inc = 0.012
yaw_rate_inc = 1.5*pi/180
angle_inc = 2*pi/180

stand_s = """Stand."""
walk_s = """Work."""
idle_s = """Rest."""

stand_flag = 0
walk_flag = 0
idle_flag = 0

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


#    def callbackv(data):

#         #global stand_flag, walk_flag, idle_flag
#         rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
#         if (data.data == stand_s):
#             stand_flag = 1
        
#         if (data.data == walk_s):
#             walk_flag = 1
        
#         if (data.data == idle_s):
#             idle_flag = 1

    def run(self):
        
        global stand_flag, walk_flag, idle_flag
        def callbackv(data):

            global stand_flag, walk_flag, idle_flag
            rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

            if (data.data == stand_s):
                stand_flag = 1
        
            if (data.data == walk_s):
                walk_flag = 1
        
            if (data.data == idle_s):
                idle_flag = 1
        
        
        self.reset_all_motion_commands_to_zero()
        #publish the stand mode
        self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)

        rospy.Subscriber('voiceWords', String, callbackv)

        while not rospy.is_shutdown():
            #print(msg)
            rospy.loginfo('wait for the commond ')
            time.sleep(1)
            if (stand_flag == 1):
                self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                print('stand mode')
                time.sleep(5)
                stand_flag = 0

            elif(idle_flag):
                self.ros_pub_idle_cmd.publish(self._idle_event_cmd_msg)
                print('idle mode')
                idle_flag = 0
            
            elif(walk_flag):

                self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                time.sleep(1)
                self.reset_all_motion_commands_to_zero()
                self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)
                print('walk mode')

                time.sleep( 0.1 )

                self._speed_cmd_msg.x = 0.05
                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s '\
                        %(self._speed_cmd_msg.x,self._speed_cmd_msg.y)) 

                time.sleep( 5 )

                self._speed_cmd_msg.x = 0
                self._speed_cmd_msg.y = 0
                self._speed_cmd_msg.z = 0

                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)
                self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)

                walk_flag = 0
            

if __name__ == "__main__":
    smkc     = SpotMicroKeyboardControl()
    smkc.run()
    