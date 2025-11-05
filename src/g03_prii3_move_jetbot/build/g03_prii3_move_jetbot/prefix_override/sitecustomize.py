import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/usuario/Proyecto3/G03_grii3_ws_ProyectoIII/src/g03_prii3_move_jetbot/install/g03_prii3_move_jetbot'
