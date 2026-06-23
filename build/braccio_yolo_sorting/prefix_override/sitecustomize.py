import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/my_workspace/src/ros2_pick_n_place/install/braccio_yolo_sorting'
