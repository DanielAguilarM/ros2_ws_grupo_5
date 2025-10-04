##velocity_limiter_node.py`
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

def clamp(x, lo, hi): return max(lo, min(hi, x))

class VelocityLimiter(Node):
    """
    Nodo intermedio (policy): sólo aplica límites.
    - /cmd_avoid (Twist) -> entrada del módulo de evasión.
    - /speed_limit (Float32, opcional) -> límite dinámico para linear.x.
    - /cmd_vel_policy (Twist) -> salida hacia el nodo de suavizado.
    """
    def __init__(self):
        super().__init__('velocity_limiter')
        # Parámetros (cargables por YAML o inline)
        self.declare_parameter('max_linear', 0.6)   # m/s
        self.declare_parameter('max_angular', 1.2)  # rad/s
        self.max_linear  = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.dynamic_speed_limit = None

        self.sub_cmd = self.create_subscription(Twist, '/cmd_avoid', self.on_cmd, 10)
        self.sub_lim = self.create_subscription(Float32, '/speed_limit', self.on_limit, 10)
        self.pub     = self.create_publisher(Twist, '/cmd_vel_policy', 10)

    def on_limit(self, msg: Float32):
        self.dynamic_speed_limit = max(0.0, float(msg.data))

    def on_cmd(self, msg_in: Twist):
        eff_lin = self.max_linear if self.dynamic_speed_limit is None \
                  else min(self.max_linear, self.dynamic_speed_limit)

        out = Twist()
        out.linear.x  = clamp(msg_in.linear.x,  -eff_lin, eff_lin)
        out.linear.y  = clamp(msg_in.linear.y,  -eff_lin, eff_lin)  # útil si la base es holonómica
        out.angular.z = clamp(msg_in.angular.z, -self.max_angular, self.max_angular)
        self.pub.publish(out)

def main():
    rclpy.init()
    node = VelocityLimiter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

