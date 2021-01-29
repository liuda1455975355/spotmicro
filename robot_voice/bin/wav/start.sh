#!/bin/bash
cd /home/liuda/spotmicro/src/spotMicro/robot_voice/bin/wav
sudo arecord -D "plughw:1,0" -d 5 -r 16000 -c 1 -t wav -f S16_LE test.wav
rostopic pub /voiceWakeup std_msgs/String "data: 'start'"
