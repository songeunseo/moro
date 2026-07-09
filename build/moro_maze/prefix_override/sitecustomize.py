import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/opt/ros2_ws/src/moro/install/moro_maze'
