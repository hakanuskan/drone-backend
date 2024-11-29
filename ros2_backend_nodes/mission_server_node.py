import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from flask import Flask, jsonify, request
from flask_cors import CORS
import threading
import asyncio


class MissionServerNode(Node):
    def __init__(self):
        super().__init__('mission_server_node')

        # MAVSDK drone sistemi
        self.drone = System()

        # Flask API
        self.app = Flask(__name__)
        CORS(self.app)

        # Flask API yolları
        self.app.add_url_rule('/api/connect', 'connect_drone', self.connect_drone, methods=['GET'])
        self.app.add_url_rule('/api/upload-mission', 'upload_mission', self.upload_mission, methods=['POST'])
        self.app.add_url_rule('/api/start-mission', 'start_mission', self.start_mission, methods=['POST'])
        self.app.add_url_rule('/api/clear-mission', 'clear_mission', self.clear_mission, methods=['POST'])
        self.app.add_url_rule('/api/download-mission', 'download_mission', self.download_mission, methods=['GET'])

        # Flask'i ayrı bir thread'de çalıştır
        self.flask_thread = threading.Thread(target=self.app.run, kwargs={'host': '0.0.0.0', 'port': 5001})
        self.flask_thread.daemon = True
        self.flask_thread.start()

        # MAVSDK bağlantı döngüsü için asyncio başlat
        self.async_loop = asyncio.new_event_loop()
        self.async_thread = threading.Thread(target=self.run_asyncio_loop)
        self.async_thread.start()

    def run_asyncio_loop(self):
        asyncio.set_event_loop(self.async_loop)
        self.async_loop.run_forever()

    async def connect_to_drone_async(self):
        """Drone bağlantısını başlatır."""
        try:
            await self.drone.connect(system_address="udp://:14540")
            self.get_logger().info("Drone connected successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to drone: {e}")

    def connect_drone(self):
        """Drone bağlantısını başlatan API."""
        asyncio.run_coroutine_threadsafe(self.connect_to_drone_async(), self.async_loop)
        return jsonify({'status': 'Drone connection initiated'}), 200

    async def upload_mission_async(self, mission_items):
        """Görevleri drone'a yükler."""
        mission_plan = MissionPlan([
            MissionItem(
                latitude_deg=item['lat'],
                longitude_deg=item['lon'],
                relative_altitude_m=item['alt'],
                speed_m_s=5.0,
                is_fly_through=True,
                gimbal_pitch_deg=0.0,
                gimbal_yaw_deg=0.0,
                camera_action=MissionItem.CameraAction.NONE
            )
            for item in mission_items
        ])
        try:
            await self.drone.mission.upload_mission(mission_plan)
            self.get_logger().info("Mission uploaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to upload mission: {e}")

    def upload_mission(self):
        """Görev yükleme API'si."""
        mission_data = request.json
        mission_items = mission_data.get('mission_items', [])
        if not mission_items:
            return jsonify({'error': 'No mission items provided'}), 400

        asyncio.run_coroutine_threadsafe(self.upload_mission_async(mission_items), self.async_loop)
        return jsonify({'status': 'Mission uploaded successfully'}), 200

    async def start_mission_async(self):
        """Görevi başlatır."""
        try:
            await self.drone.action.arm()
            await self.drone.mission.start_mission()
            self.get_logger().info("Mission started successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to start mission: {e}")

    def start_mission(self):
        """Görev başlatma API'si."""
        asyncio.run_coroutine_threadsafe(self.start_mission_async(), self.async_loop)
        return jsonify({'status': 'Mission started'}), 200

    async def download_mission_async(self):
        """Drone'dan görev listesini indirir."""
        try:
            mission_items = await self.drone.mission.download_mission()
            return [
                {
                    "lat": item.latitude_deg,
                    "lon": item.longitude_deg,
                    "alt": item.relative_altitude_m
                }
                for item in mission_items
            ]
        except Exception as e:
            self.get_logger().error(f"Failed to download mission: {e}")
            return []

    def download_mission(self):
        """Görev indirme API'si."""
        future = asyncio.run_coroutine_threadsafe(self.download_mission_async(), self.async_loop)
        mission_items = future.result()
        return jsonify(mission_items), 200

    async def clear_mission_async(self):
        """Drone'daki görevleri temizler."""
        try:
            await self.drone.mission.clear_mission()
            self.get_logger().info("Mission cleared successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to clear mission: {e}")

    def clear_mission(self):
        """Görev temizleme API'si."""
        asyncio.run_coroutine_threadsafe(self.clear_mission_async(), self.async_loop)
        return jsonify({'status': 'Mission cleared'}), 200


def main(args=None):
    rclpy.init(args=args)
    node = MissionServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
