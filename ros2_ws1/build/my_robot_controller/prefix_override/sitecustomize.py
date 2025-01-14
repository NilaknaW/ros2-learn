import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nilaunix/gitfiles/ros2learn/ros2_ws1/install/my_robot_controller'
