import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rishith/MRT/Summer_project(Morphobot)/priv-main/install/robo_control'
