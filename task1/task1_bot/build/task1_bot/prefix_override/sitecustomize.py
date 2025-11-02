import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pranabpandey31/rostasks/task1/task1_bot/install/task1_bot'
