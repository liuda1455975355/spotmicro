#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"
#include <iconv.h>
int flag=0;             //declare the flag
void receiver(const std_msgs::String::ConstPtr& msg)
{
    std::cout<<"I heard :"<<msg->data.c_str()<<std::endl;      //print
    flag=1;               //set flag 1
}
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "receiverrr");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("voiceWords", 1000, receiver);
    ros::spin();
   

    return 0;
}