import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/abhimanyu/nav2_ws/src/lirovo/install/lirovo'
