import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lizhen/works/open_source/model_convert/ros2_tools/motion_editor/install/motion_editor'
