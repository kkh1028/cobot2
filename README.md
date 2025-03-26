# cobot2

"""
colcon build --symlink-install

source install /sdetup.bash

export PYTHONPATH=$PYTHONPATH:~/ros2_ws/install/common2/lib/common2/imp

ping 192.168.1.100
ifconfig


ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609

ros2 service list | grep tool
ros2 service type /dsr01/tool/get_current_tool
ros2 service call /dsr01/tool/get_current_tool dsr_msgs2/srv/GetCurrentTool 
ros2 run rokey simple_move
"""
