import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/oli/workspace/ros2_ws/install/mentorpi_description'
