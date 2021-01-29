#!/usr/bin/python
"""
Class for sending keyboard commands to spot micro walk node, control velocity, yaw rate, and walk event 
"""
import rospy
import sched, time
import sys, select, termios, tty # For terminal keyboard key press reading
from std_msgs.msg import Float32, Bool 
from geometry_msgs.msg import Vector3
from math import pi

msg = """
 Walk_test  Command

Enter one of the following options:
-----------------------------
quit: stop and quit the program
walk: Start walk mode
walkf: Start walk mode ,walk forward 10s,speed 0.05
walkb: Start walk mode ,walk backward 10s,speed -0.05
stand: Stand robot up
idle: relax the servo
anything else : Prompt again for command

CTRL-C to quit
"""
valid_cmds = ('quit','walk','walkb','stand','idle')
# Global body motion increment values
speed_inc = 0.012
yaw_rate_inc = 1.5*pi/180
angle_inc = 2*pi/180

class SpotMicroKeyboardControl():
    def __init__(self):

        # Create messages for body motion commands, and initialize to zero 
        self._x_speed_cmd_msg = Float32()
        self._x_speed_cmd_msg.data = 0

        self._y_speed_cmd_msg = Float32()
        self._y_speed_cmd_msg.data = 0

        self._yaw_rate_cmd_msg = Float32()
        self._yaw_rate_cmd_msg.data = 0

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

        rospy.loginfo("Setting Up the Spot Micro Keyboard Control Node...")

        # Set up and title the ros node for this code
        rospy.init_node('spot_micro_keyboard_control')

        # Create publishers for x,y speed command, body rate command, and state command
        self.ros_pub_x_speed_cmd    = rospy.Publisher('x_speed_cmd',Float32,queue_size=1)
        self.ros_pub_y_speed_cmd    = rospy.Publisher('/y_speed_cmd',Float32,queue_size=1)
        self.ros_pub_yaw_rate_cmd   = rospy.Publisher('/yaw_rate_cmd',Float32,queue_size=1)
        self.ros_pub_speed_cmd      = rospy.Publisher('/speed_cmd',Vector3,queue_size=1)
        self.ros_pub_state_cmd      = rospy.Publisher('/state_cmd',Bool,queue_size=1)
        self.ros_pub_walk_cmd       = rospy.Publisher('/walk_cmd',Bool, queue_size=1)
        self.ros_pub_stand_cmd      = rospy.Publisher('/stand_cmd',Bool,queue_size=1)
        self.ros_pub_idle_cmd       = rospy.Publisher('/idle_cmd',Bool,queue_size=1)

        rospy.loginfo("> Publishers corrrectly initialized")

        rospy.loginfo("Initialization complete")

        # Setup terminal input reading, taken from teleop_twist_keyboard
        self.settings = termios.tcgetattr(sys.stdin)

    def reset_all_motion_commands_to_zero(self):
        '''Reset body motion cmd states to zero and publish zero value body motion commands'''
        self._x_speed_cmd_msg.data = 0
        self._y_speed_cmd_msg.data = 0
        self._yaw_rate_cmd_msg.data = 0

        self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)
        self.ros_pub_y_speed_cmd.publish(self._y_speed_cmd_msg)
        self.ros_pub_yaw_rate_cmd.publish(self._yaw_rate_cmd_msg)

        self._speed_cmd_msg.x = 0
        self._speed_cmd_msg.y = 0
        self._speed_cmd_msg.z = 0

        self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

    
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        # Publish all body motion commands to 0
        self.reset_all_motion_commands_to_zero()

        # Prompt user with keyboard command information
        
        while not rospy.is_shutdown():
            print(msg)
            userInput = raw_input("Command?: ")

            if userInput not in valid_cmds:
                print('Valid command not entered, try again...')
            else:
                if userInput == 'quit':
                    print("Ending program...")
                    break
                
                elif userInput == 'stand':
                    #Publish stand command event
                    self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                
                elif userInput == 'idle':
                    #Publish idle command event
                    self.ros_pub_idle_cmd.publish(self._idle_event_cmd_msg)

                elif userInput == 'walk':
                    # Reset all body motion commands to zero
                    self.reset_all_motion_commands_to_zero()

                    # Publish walk event
                    #self.ros_pub_state_cmd.publish(self._walk_event_cmd_msg)
                    #self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)
                    print('Enter command, u to go back to stand mode: ')

                    '''
                    self._x_speed_cmd_msg.data = 0.05
                    self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)

                    self._speed_cmd_msg.x = 0.05
                    self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                    print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s '\
                                %(self._x_speed_cmd_msg.data,self._y_speed_cmd_msg.data,self._yaw_rate_cmd_msg.data*180/pi))            
                    print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s '\
                                %(self._speed_cmd_msg.x,self._speed_cmd_msg.y,self._yaw_rate_cmd_msg.data*180/pi))  

                    time.sleep( 10 )

                    self._speed_cmd_msg.x = 0
                    self._speed_cmd_msg.y = 0
                    self._speed_cmd_msg.z = 0
                    self._x_speed_cmd_msg.data = 0
                    self._y_speed_cmd_msg.data = 0
                    self._yaw_rate_cmd_msg.data = 0

                    self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)
                    # Send walk event message, this will take robot back to rest mode
                    self.ros_pub_state_cmd.publish(self._walk_event_cmd_msg)
                    self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                
                    
                
                elif userInput == 'walkb':
                    # Reset all body motion commands to zero
                    self.reset_all_motion_commands_to_zero()

                    # Publish walk event
                    #self.ros_pub_state_cmd.publish(self._walk_event_cmd_msg)
                    self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)

                    #self._x_speed_cmd_msg.data = -0.05
                    #self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)
                    while (1):
                        self._speed_cmd_msg.x = -0.05
                        self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                        print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s '\
                                %(self._x_speed_cmd_msg.data,self._y_speed_cmd_msg.data,self._yaw_rate_cmd_msg.data*180/pi))            
                        print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s '\
                                %(self._speed_cmd_msg.x,self._speed_cmd_msg.y,self._yaw_rate_cmd_msg.data*180/pi))  

                        time.sleep( 10 )

                        self._speed_cmd_msg.x = 0
                        self._speed_cmd_msg.y = 0
                        self._speed_cmd_msg.z = 0
                        self._x_speed_cmd_msg.data = 0
                        self._y_speed_cmd_msg.data = 0
                        self._yaw_rate_cmd_msg.data = 0

                        self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)
                        # Send walk event message, this will take robot back to rest mode
                        #self.ros_pub_state_cmd.publish(self._walk_event_cmd_msg)
                        self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)                    
                    '''
                  

                    while (1):
                        print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s '\
                                %(self._x_speed_cmd_msg.data,self._y_speed_cmd_msg.data,self._yaw_rate_cmd_msg.data*180/pi))
                        print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s '\
                                %(self._speed_cmd_msg.x,self._speed_cmd_msg.y,self._yaw_rate_cmd_msg.data*180/pi)) 

                        userInput = self.getKey()

                        if userInput == 'u':
                            # Send walk event message, this will take robot back to rest mode
                            self.ros_pub_state_cmd.publish(self._walk_event_cmd_msg)

                            self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                            break

                        elif userInput not in ('w','a','s','d','q','e','u','f','i','k'):
                            print('Key not in valid key commands, try again')
                        else:
                            if userInput == 'w':
                                self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)

                                self._x_speed_cmd_msg.data = self._x_speed_cmd_msg.data + speed_inc    
                                self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)

                                self._speed_cmd_msg.x = self._speed_cmd_msg.x + speed_inc
                                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)
                            
                            elif userInput == 's':
                                self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)

                                self._x_speed_cmd_msg.data = self._x_speed_cmd_msg.data - speed_inc
                                self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)

                                self._speed_cmd_msg.x = self._speed_cmd_msg.x - speed_inc
                                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                            elif userInput == 'a':
                                self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)

                                self._y_speed_cmd_msg.data = self._y_speed_cmd_msg.data + speed_inc
                                self.ros_pub_y_speed_cmd.publish(self._y_speed_cmd_msg)

                                self._speed_cmd_msg.y = self._speed_cmd_msg.y - speed_inc
                                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)
                            
                            elif userInput == 'd':
                                self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)

                                self._y_speed_cmd_msg.data = self._y_speed_cmd_msg.data - speed_inc
                                self.ros_pub_y_speed_cmd.publish(self._y_speed_cmd_msg)

                                self._speed_cmd_msg.y = self._speed_cmd_msg.y + speed_inc
                                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                            elif userInput == 'q':
                                self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)

                                self._yaw_rate_cmd_msg.data = self._yaw_rate_cmd_msg.data + yaw_rate_inc
                                self.ros_pub_yaw_rate_cmd.publish(self._yaw_rate_cmd_msg)

                                self._speed_cmd_msg.z = self._speed_cmd_msg.z - yaw_rate_inc
                                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                            elif userInput == 'e':
                                self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)

                                self._yaw_rate_cmd_msg.data = self._yaw_rate_cmd_msg.data - yaw_rate_inc
                                self.ros_pub_yaw_rate_cmd.publish(self._yaw_rate_cmd_msg)

                                self._speed_cmd_msg.z = self._speed_cmd_msg.z + yaw_rate_inc
                                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                            elif userInput == 'f':
                                self._speed_cmd_msg.x = 0
                                self._speed_cmd_msg.y = 0
                                self._speed_cmd_msg.z = 0
                                self._x_speed_cmd_msg.data = 0
                                self._y_speed_cmd_msg.data = 0
                                self._yaw_rate_cmd_msg.data = 0

                                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                            elif userInput == 'i':
                                self.ros_pub_state_cmd.publish(self._walk_event_cmd_msg)
                                self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)

                                time.sleep( 0.1 )

                                self._x_speed_cmd_msg.data = 0.05
                                self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)

                                self._speed_cmd_msg.x = 0.05
                                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                                print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s '\
                                %(self._speed_cmd_msg.x,self._speed_cmd_msg.y,self._yaw_rate_cmd_msg.data*180/pi)) 

                                time.sleep( 10 )

                                self._speed_cmd_msg.x = 0
                                self._speed_cmd_msg.y = 0
                                self._speed_cmd_msg.z = 0
                                self._x_speed_cmd_msg.data = 0
                                self._y_speed_cmd_msg.data = 0
                                self._yaw_rate_cmd_msg.data = 0

                                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)
                                self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                            

                            elif userInput == 'k':
                                self.ros_pub_state_cmd.publish(self._walk_event_cmd_msg)
                                self.ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)

                                time.sleep( 0.1 )

                                self._x_speed_cmd_msg.data = -0.05
                                self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)

                                self._speed_cmd_msg.x = -0.05
                                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)

                                print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s '\
                                %(self._speed_cmd_msg.x,self._speed_cmd_msg.y,self._yaw_rate_cmd_msg.data*180/pi)) 

                                time.sleep( 10 )

                                self._speed_cmd_msg.x = 0
                                self._speed_cmd_msg.y = 0
                                self._speed_cmd_msg.z = 0
                                self._x_speed_cmd_msg.data = 0
                                self._y_speed_cmd_msg.data = 0
                                self._yaw_rate_cmd_msg.data = 0

                                self.ros_pub_speed_cmd.publish(self._speed_cmd_msg)
                                self.ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                                
                          

if __name__ == "__main__":
    smkc     = SpotMicroKeyboardControl()
    smkc.run()