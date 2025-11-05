import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool

class MoveJetbot(Node):
    def __init__(self):
        super().__init__('g03_prii3_move_jetbot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_callback)

        # Servicio para pausar/reanudar
        self.srv = self.create_service(SetBool, 'pause_or_resume_move', self.srv_callback)
        
        self.is_paused = False
        self.step = 0
        self.get_logger().info('Nodo MoveJetbot iniciado (Grupo 3)')

    def srv_callback(self, request, response):
        if request.data:  # True → pausa
            self.is_paused = True
            self.get_logger().info('Movimiento pausado')
            response.success = True
            response.message = "Movimiento pausado"
        else:  # False → reanuda
            self.is_paused = False
            self.get_logger().info('Movimiento reanudado')
            response.success = True
            response.message = "Movimiento reanudado"
        return response

    def move_callback(self):
	    if self.is_paused:
		return

	    twist = Twist()

	    # Parámetros ajustables
	    v = 0.25   # velocidad lineal (m/s)
	    w = 0.6    # velocidad angular (rad/s)
	    t_forward = 6   # pasos rectos (0.5 s cada uno)
	    t_turn_90 = 3   # pasos para giro de 90°
	    t_turn_180 = 6  # pasos para giro de 180° (el doble de 90°)

	    # Secuencia del “3” cuadrado:
	    # recto → 90° → recto → 90° → recto → 180° → recto → 90° → recto → 90° → recto
	    if self.step < t_forward:                                  # 1 recto
		twist.linear.x = v
	    elif self.step < t_forward + t_turn_90:                    # 2 giro 90°
		twist.angular.z = -w
	    elif self.step < t_forward * 2 + t_turn_90:                # 3 recto
		twist.linear.x = v
	    elif self.step < t_forward * 2 + t_turn_90 * 2:            # 4 giro 90°
		twist.angular.z = -w
	    elif self.step < t_forward * 3 + t_turn_90 * 2:            # 5 recto
		twist.linear.x = v
	    elif self.step < t_forward * 3 + t_turn_90 * 2 + t_turn_180:  # 6 giro 180°
		twist.angular.z = -w
	    elif self.step < t_forward * 4 + t_turn_90 * 2 + t_turn_180:  # 7 recto
		twist.linear.x = v
	    elif self.step < t_forward * 4 + t_turn_90 * 3 + t_turn_180:  # 8 giro 90°
		twist.angular.z = -w
	    elif self.step < t_forward * 5 + t_turn_90 * 3 + t_turn_180:  # 9 recto
		twist.linear.x = v
	    elif self.step < t_forward * 5 + t_turn_90 * 4 + t_turn_180:  # 10 giro 90°
		twist.angular.z = -w
	    elif self.step < t_forward * 6 + t_turn_90 * 4 + t_turn_180:  # 11 recto final
		twist.linear.x = v
	    else:                                                        # stop
		twist.linear.x = 0.0
		twist.angular.z = 0.0

	    # Publicar comando
	    self.publisher_.publish(twist)
	    self.step += 1

	    # Reinicio automático
	    total_steps = t_forward * 6 + t_turn_90 * 4 + t_turn_180 + 5
	    if self.step >= total_steps:
		self.get_logger().info('Trayectoria “3” completada. Reiniciando.')
		self.step = 0


def main(args=None):
    rclpy.init(args=args)
    node = MoveJetbot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



