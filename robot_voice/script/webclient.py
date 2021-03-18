from websocket import create_connection
import time
import rospy
from std_msgs.msg import String
voice_ws = create_connection("ws://192.168.8.188:8540/sendDestination")

dest1="715"
dest2="716"

flag_715 = 0
flag_716 = 0

class spotmicrovoicewebsocket():
    def __init__(self):
        rospy.loginfo("Setting up the SpotMicro voice websocket client node ...")
        rospy.init_node('spot_micro_voice_websocket_client')
        rospy.loginfo("Initialization complete")

    def run(self):

        global flag_715, flag_716
        def callback(data):
            global flag_715, flag_716
            rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

            if (data.data == dest1):
                flag_715 = 1
            if (data.data == dest2):
                flag_716 = 1

        rospy.Subscriber('voiceWords',String, callback)

        while not rospy.is_shutdown():
            rospy.loginfo('wait for the command')

            if (flag_715 ==1):
                voice_ws.send("715")
                time.sleep(1)
                result=voice_ws.recv()
                print(result)
                flag_715 =0
            
            elif (flag_716 ==1):
                voice_ws.send("716")
                time.sleep(1)
                result=voice_ws.recv()
                print(result)
                flag_716=0

            else:
                voice_ws.send("nothing")
                time.sleep(1)
                result=voice_ws.recv()
                print(result)


            
if __name__ =="__main__":
    smkc = spotmicrovoicewebsocket()
    smkc.run()




