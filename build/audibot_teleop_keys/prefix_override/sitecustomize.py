import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tejas_bhuta/git/drive_to_survive/install/audibot_teleop_keys'
