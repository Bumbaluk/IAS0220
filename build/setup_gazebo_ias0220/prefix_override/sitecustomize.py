import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lukas/ros2_ws/install/setup_gazebo_ias0220'
