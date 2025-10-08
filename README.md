
PASOS PARA HACER FUNCIONAR LA APLICACIÓN

1-Abrir una terminal y situarse en el workspace:
    cd ~/Proyecto3/g03_prii3_ws

2-Cargar el entorno de ROS2 y del workspace:
    source /opt/ros/foxy/setup.bash
    source install/setup.bash

3-Compilar el proyecto (solo si hay cambios nuevos):
    colcon build --symlink-install

3-Lanzar el simulador y el nodo automático:
    ros2 launch g03_prii3_turtlesim auto_turtle_launch.py
  O con dos terminales:
    ros2 run turtlesim turtlesim_node
    ros2 run g03_prii3_turtlesim auto_turtle

------------------------------------------------------------
 COMANDOS DE CONTROL DE LA TORTUGA

Pausar el movimiento:
    ros2 service call /pause_or_resume example_interfaces/srv/SetBool "{data: true}"

Reanudar el movimiento:
    ros2 service call /pause_or_resume example_interfaces/srv/SetBool "{data: false}"

Reiniciar el dibujo:
    ros2 service call /restart_drawing std_srvs/srv/Empty "{}"

