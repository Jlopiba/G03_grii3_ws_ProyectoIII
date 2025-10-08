import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/usuario/Proyecto3/g03_prii3_ws/install/g03_prii3_turtlesim'
