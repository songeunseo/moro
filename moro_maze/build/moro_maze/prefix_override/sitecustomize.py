import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/opt/ros2_ws/src/moro/moro_maze/install/moro_maze'
