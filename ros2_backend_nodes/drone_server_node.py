import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import BatteryStatus, VehicleStatus, VehicleOdometry, VehicleGlobalPosition, VehicleCommand
from flask import Flask, jsonify, request
from flask_cors import CORS
import threading
from datetime import datetime

class DroneTelemetryNode(Node):
    def __init__(self):
        super().__init__('drone_telemetry_node')
        # Initialize Flask app
        self.app = Flask(__name__)
        CORS(self.app)  # Enable CORS for the Flask app
        self.app.add_url_rule('/api/data', 'get_data', self.get_data)
        self.app.add_url_rule('/api/return_to_launch', 'send_return_to_launch_command', self.send_return_to_launch_command, methods=['POST'])
        # Start Flask app in a separate thread
        self.flask_thread = threading.Thread(target=self.app.run, kwargs={'host': '0.0.0.0', 'port': 5000})
        self.flask_thread.daemon = True
        self.flask_thread.start()

        # Define QoS profile with Best Effort reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Initialize data storage
        self.last_battery = {}
        self.last_vehicle_status = {}
        self.last_vehicle_odometry = {}
        self.last_global_position = {}
        self.current_mission = []

        # Subscribe to PX4 topics with the specified QoS profile
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self.battery_callback, qos_profile)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.global_position_callback, qos_profile)
        self.command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)


    async def connect_drone(self):
        try:
            await self.drone.connect(system_address="udp://:14540")
            print("Drone connected")
        except Exception as e:
            print(f"Failed to connect to drone: {e}")

    def send_return_to_launch_command(self):
        command_msg = VehicleCommand()
        command_msg.command = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        command_msg.target_system = 1
        command_msg.target_component = 1
        command_msg.source_system = 1
        command_msg.source_component = 1
        command_msg.from_external = True
        self.command_publisher.publish(command_msg)
        self.get_logger().info("Return to Launch command sent")
        return jsonify({'status': 'Return to Launch command sent'})

    def battery_callback(self, msg):
        self.last_battery = {"battery": float(msg.remaining)}

    def vehicle_status_callback(self, msg):
        self.last_vehicle_status = {
            "arming_state": msg.arming_state,
            "nav_state": msg.nav_state
        }

    def vehicle_odometry_callback(self, msg):
        self.last_vehicle_odometry = {
            "position": {
                "x": float(msg.position[0]),
                "y": float(msg.position[1]),
                "z": float(msg.position[2])
            },
            "velocity": {
                "x": float(msg.velocity[0]),
                "y": float(msg.velocity[1]),
                "z": float(msg.velocity[2])
            }
        }

    def global_position_callback(self, msg):
        self.last_global_position = {
            "lat": float(msg.lat),  # Latitude in degrees
            "lon": float(msg.lon),  # Longitude in degrees
            "alt": float(msg.alt),  # Altitude in meters
            "eph": float(msg.eph),  # Standard deviation of horizontal position error in meters
            "epv": float(msg.epv)   # Standard deviation of vertical position error in meters
        }

    def get_data(self):
        data = {
            "timestamp": datetime.utcnow().isoformat() + 'Z',
            "battery": self.last_battery,
            "vehicle_status": self.last_vehicle_status,
            "vehicle_odometry": self.last_vehicle_odometry,
            "global_position": self.last_global_position
        }
        return jsonify(data)

def main(args=None):
    rclpy.init(args=args)
    node = DroneTelemetryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()