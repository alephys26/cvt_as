import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/parallels/Desktop/auto_assign1/cvt_as/mscvt/src/mscvt_ros/install/mscvt_ros'
