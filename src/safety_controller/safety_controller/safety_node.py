import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')

        # Suscriptores
        self.create_subscription(Twist, 'cmd_policy', self.cmd_policy_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(Bool, 'emergency_stop', self.emergency_callback, 10)

        # Publicador final
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Estados internos
        self.latest_cmd = Twist()
        self.obstacle_detected = False
        self.emergency_stop = False

    def cmd_policy_callback(self, msg):
        self.latest_cmd = msg
        self.evaluate_and_publish()

    def scan_callback(self, msg: LaserScan):
        # Ver si hay un objeto a <0.5m
        if any(r < 0.5 for r in msg.ranges if r > 0.0):
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
        self.evaluate_and_publish()

    def emergency_callback(self, msg: Bool):
        self.emergency_stop = msg.data
        self.evaluate_and_publish()

    def evaluate_and_publish(self):
        output = Twist()

        if self.emergency_stop:
            # Parada total
            output.linear.x = 0.0
            output.angular.z = 0.0
        elif self.obstacle_detected:
            # Obstáculo cerca → detener
            output.linear.x = 0.0
            output.angular.z = self.latest_cmd.angular.z
        else:
            # Pasar comando directo
            output = self.latest_cmd

        self.cmd_vel_pub.publish(output)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
