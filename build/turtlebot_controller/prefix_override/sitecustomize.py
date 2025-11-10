import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/leo/Desktop/turtlebot_workspace_vocale/install/turtlebot_controller'
