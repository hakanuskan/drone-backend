import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import redis
import json
import time

class GPSListener(Node):
    def __init__(self):
        super().__init__('gps_listener')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Redis
        try:
            self.redis_client = redis.StrictRedis(
                host='redis-10894.c311.eu-central-1-1.ec2.redns.redis-cloud.com',
                port=10894,
                password='eRy4RfZjLYCSheuYiJE3Ha949qvqjPNg',
                ssl=False
            )
            self.redis_client.ping()
            self.get_logger().info("Redis bağlantısı başarılı.")
        except redis.ConnectionError:
            self.get_logger().error("Redis sunucusuna bağlanılamıyor.")
            return
        
        self.subscription = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.listener_callback,
            qos_profile
        )
        
    def listener_callback(self, msg):
        gps_data = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'timestamp': time.time()
        }
        self.get_logger().info(f"GPS Data: {gps_data}")

        try:
            self.redis_client.set("gps_data", json.dumps(gps_data))
            self.get_logger().info("GPS verileri Redis Cloud'a yazıldı.")
        except redis.RedisError as e:
            self.get_logger().error(f"Redis Cloud'a veri yazma hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

