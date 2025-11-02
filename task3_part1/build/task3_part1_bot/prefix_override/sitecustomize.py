import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pranabpandey31/rostasks/task3_part1/install/task3_part1_bot'
