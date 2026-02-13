#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class EvasionSimple(Node):
    def __init__(self):
        super().__init__('evasion_simple')
        
        self.distancia_limite = 0.03
        
        self.direccion_actual = "BUSCANDO"
        
        self.publisher = self.create_publisher(Twist, '/cmd_evation', 10)
        
        self.subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.laser_callback, 
            10
        )
        
    def laser_callback(self, msg):
        obstaculo = self.detectar_obstaculo(msg)
        
        cmd = Twist()
        
        if obstaculo:
            if self.direccion_actual == "BUSCANDO":
                self.direccion_actual = "DERECHA"
            elif self.direccion_actual == "DERECHA":
                self.direccion_actual = "IZQUIERDA"
            else:
                self.direccion_actual = "BUSCANDO"
            
            if self.direccion_actual == "DERECHA":
                cmd.angular.z = -1.0
            elif self.direccion_actual == "IZQUIERDA":
                cmd.angular.z = 1.0
                
        else:
            self.direccion_actual = "BUSCANDO"
            cmd.angular.z = 0.0
        
        self.publisher.publish(cmd)
    
    def detectar_obstaculo(self, msg):
        if not msg.ranges:
            return False
            
        total = len(msg.ranges)
        inicio = int(total * 0.35)
        fin = int(total * 0.65)
        
        for i in range(inicio, fin):
            if 0.001 < msg.ranges[i] < self.distancia_limite:
                return True
                
        return False

def main(args=None):
    rclpy.init(args=args)
    node = EvasionSimple()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
