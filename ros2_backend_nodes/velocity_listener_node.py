import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import redis
import json
import time

class VelocityListener(Node):
    def __init__(self):
        super().__init__('velocity_listener')
        
        self.redis_client = redis.StrictRedis(host='localhost', port=6379, db=0)
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.listener_callback,
            qos_profile 
        )

    def listener_callback(self, msg):
        velocity_data = {
            'linear_x': msg.twist.linear.x,
            'linear_y': msg.twist.linear.y,
            'linear_z': msg.twist.linear.z,
            'angular_x': msg.twist.angular.x,
            'angular_y': msg.twist.angular.y,
            'angular_z': msg.twist.angular.z,
            'timestamp': time.time()
        }
        self.get_logger().info(f"Velocity Data: {velocity_data}")

        self.redis_client.set("velocity_data", json.dumps(velocity_data))

def main(args=None):
    rclpy.init(args=args)
    node = VelocityListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
