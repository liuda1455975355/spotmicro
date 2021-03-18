完整的机器狗控制命令
1.开启中控服务端
source robot/bin/activate
python run.py
http://127.0.0.1:5000/demo2
点击“生成图形”
点击“启动服务”
2.开启数莓派snowboy语音唤醒
sudo -s
roslaunch snowboy_wakeup snowboy_wakeup.launch
3.启动机器狗控制部分启动文件
roslaunch robot_voice spot_voice_walk.launch
