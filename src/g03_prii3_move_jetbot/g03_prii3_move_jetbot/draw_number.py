import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import time

class DrawNumber(Node):
    def __init__(self):
        super().__init__('draw_number')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.srv = self.create_service(SetBool, 'toggle_drawing', self.service_callback)
        self.active = True
        self.get_logger().info('Nodo draw_number iniciado')

    def service_callback(self, request, response):
        # Si request.data == True → reanudar, False → parar
        self.active = request.data
        response.success = True
        response.message = f"Dibujo {'reanuda' if self.active else 'detiene'}"
        self.get_logger().info(response.message)
        return response

    def move(self, linear, angular, t):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        end_time = self.get_clock().now().to_msg().sec + t
        while self.get_clock().now().to_msg().sec < end_time and rclpy.ok():
            if not self.active:
                continue
            self.publisher_.publish(msg)
            time.sleep(0.1)
        self.publisher_.publish(Twist())  # parar

    def draw_pattern(self):
        # Ejemplo: moverse hacia delante y girar (ajusta para tu número)
        while rclpy.ok():
            if self.active:
                self.get_logger().info("Dibujando número...")
                self.move(0.4, 0.0, 3)   # recta
                self.move(0.0, 1.0, 2.5) # giro
                self.move(0.2, 0.4, 3)
                self.publisher_.publish(Twist())  # stop
                break

def main(args=None):
    rclpy.init(args=args)
    node = DrawNumber()

    # Crea un timer que llama a draw_pattern solo una vez
    node.create_timer(1.0, node.draw_pattern)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

