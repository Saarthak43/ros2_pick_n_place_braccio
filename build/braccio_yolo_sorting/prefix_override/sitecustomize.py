import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/project1/src/ros2_pick_n_place_braccio/install/braccio_yolo_sorting'
