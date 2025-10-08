#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from example_interfaces.srv import SetBool


class AutoTurtle(Node):
    def __init__(self):
        super().__init__('auto_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_pattern)
        self.step = 0
        self.paused = False

        # Servicios
        self.pause_srv = self.create_service(SetBool, 'pause_or_resume', self.pause_or_resume_callback)
        self.reset_srv = self.create_service(Empty, 'restart_drawing', self.restart_drawing_callback)

        self.get_logger().info('Nodo auto_turtle iniciado ')

    def move_pattern(self):
        # Si está pausado, no se mueve
        if self.paused:
            return

        msg = Twist()

        if 0 <= self.step < 25:
            msg.linear.x = 1.0
        elif 25 <= self.step < 35:
            msg.angular.z = 1.6
        elif 35 <= self.step < 60:
            msg.linear.x = 1.0
        elif 60 <= self.step < 70:
            msg.angular.z = 1.6
        elif 70 <= self.step < 95:
            msg.linear.x = 1.0
        elif 95 <= self.step < 115:
            msg.angular.z = 1.55
        elif 115 <= self.step < 140:
            msg.linear.x = 1.0
        elif 140 <= self.step < 150:
            msg.angular.z = -1.65
        elif 150 <= self.step < 195:
            msg.linear.x = 1.0
        elif 195 <= self.step < 205:
            msg.angular.z = -1.5
        elif 205 <= self.step < 225:
            msg.linear.x = 1.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)
        self.step += 1

    def pause_or_resume_callback(self, request, response):
        self.paused = request.data
        response.success = True
        response.message = "Movimiento pausado" if self.paused else "Movimiento reanudado"
        self.get_logger().info(response.message)
        return response

    def restart_drawing_callback(self, request, response):
        self.step = 0
        self.paused = False
        self.get_logger().info("Dibujo reiniciado desde el principio.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AutoTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
