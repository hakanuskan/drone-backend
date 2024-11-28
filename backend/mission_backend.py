from flask import Flask, jsonify, request, stream_with_context, Response
from flask_cors import CORS
import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from datetime import datetime
import math
 
app = Flask(__name__)
CORS(app)  # Enable CORS for all routes
drone = System()
 
# Variables to store the last known good data
last_battery = {"battery": None}
last_gps_info = {"num_satellites": None, "fix_type": None}
last_in_air = {"in_air": None}
last_position = {
    "latitude_deg": None,
    "longitude_deg": None,
    "absolute_altitude_m": None,
    "relative_altitude_m": None
}
 
async def connect_drone():
    try:
        await drone.connect(system_address="udp://:14540")
        print("Drone connected")
    except Exception as e:
        print(f"Failed to connect to drone: {e}")
        
async def notify_qgc_about_mission(drone):
    try:
        # Görev listesini indir ve QGC'ye bildir
        mission_items = await drone.mission.download_mission()
        for i, item in enumerate(mission_items.mission_items):
            # Görevlerin her birini QGC'ye bildir
            print(f"Sending mission item {i+1} to QGC")
            await drone.mission.set_current_mission_item(i)
        print("All mission items sent to QGC.")
    except Exception as e:
        print(f"Failed to notify QGroundControl: {e}")
 
async def return_to_launch_pos(drone):
    try:
        await drone.action.return_to_launch()
        print("Returning to launch")
    except Exception as e:
        print(f"Failed to return to launch: {e}")
 
async def get_mission(drone):
    try:
        response = await drone.mission.download_mission()
        print("Mission downloaded")
        return response
    except Exception as e:
        print(f"Failed to download mission: {e}")
        return None
 
async def upload_mission(drone, mission_plan):
    try:
        await drone.mission.upload_mission(mission_plan)
        print("Mission uploaded")
    except Exception as e:
        print(f"Failed to upload mission: {e}")
 
async def upload_mission_prog(drone, mission_plan):
    try:
        async for progress in drone.mission.upload_mission_with_progress(mission_plan):
            yield f"data: Mission upload progress: {progress}\n\n"
        yield "data: Mission uploaded with progress\n\n"
    except Exception as e:
        yield f"data: Failed to upload mission with progress: {e}\n\n"
 
def mission_plan_to_dict(mission_plan):
    def safe_value(value):
        return None if isinstance(value, float) and math.isnan(value) else value
 
    return {
        "mission_items": [
            {
                "latitude_deg": item.latitude_deg,
                "longitude_deg": item.longitude_deg,
                "relative_altitude_m": item.relative_altitude_m,
                "speed_m_s": safe_value(item.speed_m_s),
                "is_fly_through": item.is_fly_through,
                "gimbal_pitch_deg": safe_value(item.gimbal_pitch_deg),
                "gimbal_yaw_deg": safe_value(item.gimbal_yaw_deg),
                "camera_action": item.camera_action.name,
                "loiter_time_s": safe_value(item.loiter_time_s),
                "camera_photo_interval_s": safe_value(item.camera_photo_interval_s),
                "acceptance_radius_m": safe_value(item.acceptance_radius_m),
                "yaw_deg": safe_value(item.yaw_deg),
                "camera_photo_distance_m": safe_value(item.camera_photo_distance_m),
                "vehicle_action": item.vehicle_action.name,
            }
            for item in mission_plan.mission_items
        ]
    }
 
def dict_to_mission_plan(data):
    mission_items = [
        MissionItem(
            latitude_deg=item["latitude_deg"],
            longitude_deg=item["longitude_deg"],
            relative_altitude_m=item["relative_altitude_m"],
            speed_m_s=item.get("speed_m_s", 0.0),  # Default to 0.0 if None
            is_fly_through=item["is_fly_through"],
            gimbal_pitch_deg=item.get("gimbal_pitch_deg", 0.0),  # Default to 0.0 if None
            gimbal_yaw_deg=item.get("gimbal_yaw_deg", 0.0),  # Default to 0.0 if None
            camera_action=MissionItem.CameraAction[item.get("camera_action", "NONE")],  # Default to 'NONE' if None
            loiter_time_s=item.get("loiter_time_s", 0.0),  # Default to 0.0 if None
            camera_photo_interval_s=item.get("camera_photo_interval_s", 0.0),  # Default to 0.0 if None
            acceptance_radius_m=item.get("acceptance_radius_m", 0.0),  # Default to 0.0 if None
            yaw_deg=item.get("yaw_deg", 0.0),  # Default to 0.0 if None
            camera_photo_distance_m=item.get("camera_photo_distance_m", 0.0),  # Default to 0.0 if None
            vehicle_action=MissionItem.VehicleAction[item.get("vehicle_action", "NONE")]  # Default to 'NONE' if None
        )
        for item in data["mission_items"]
    ]
    return MissionPlan(mission_items)
 
async def print_battery(drone):
    global last_battery
    try:
        async for battery in drone.telemetry.battery():
            print(f"Battery: {battery.remaining_percent}")
            last_battery = {"battery": battery.remaining_percent}
            return last_battery
    except Exception as e:
        print(f"Failed to get battery data: {e}")
        return last_battery
 
async def print_gps_info(drone):
    global last_gps_info
    try:
        async for gps_info in drone.telemetry.gps_info():
            print(f"GPS info: {gps_info}")
            last_gps_info = {
                "num_satellites": gps_info.num_satellites,
                "fix_type": gps_info.fix_type.name  # Convert FixType to string
            }
            return last_gps_info
    except Exception as e:
        print(f"Failed to get GPS info: {e}")
        return last_gps_info
 
async def print_in_air(drone):
    global last_in_air
    try:
        async for in_air in drone.telemetry.in_air():
            print(f"In air: {in_air}")
            last_in_air = {"in_air": in_air}
            return last_in_air
    except Exception as e:
        print(f"Failed to get in air status: {e}")
        return last_in_air
 
async def print_position(drone):
    global last_position
    try:
        async for position in drone.telemetry.position():
            print(f"Position: {position}")
            last_position = {
                "latitude_deg": position.latitude_deg,
                "longitude_deg": position.longitude_deg,
                "absolute_altitude_m": position.absolute_altitude_m,
                "relative_altitude_m": position.relative_altitude_m
            }
            return last_position
    except Exception as e:
        print(f"Failed to get position data: {e}")
        return last_position
 
@app.route('/')
def home():
    return "Hello, Flask!"
 
@app.route('/api/data', methods=['GET'])
def get_data():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(connect_drone())
    battery = loop.run_until_complete(print_battery(drone))
    gps_info = loop.run_until_complete(print_gps_info(drone))
    in_air = loop.run_until_complete(print_in_air(drone))
    position = loop.run_until_complete(print_position(drone))
    data = {
        "timestamp": datetime.utcnow().isoformat() + 'Z',  # Add timestamp in ISO format
        "battery": battery,
        "gps_info": gps_info,
        "in_air": in_air,
        "position": position
    }
    print(f"Data to be returned: {data}")  # Log the data being returned
    return jsonify(data)
 
@app.route('/api/return', methods=['POST'])
def return_to_launch():
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(connect_drone())
        loop.run_until_complete(return_to_launch_pos(drone))
        return jsonify("Returning to launch"), 201
    except Exception as e:
        print(f"Failed to return to launch: {e}")
        return jsonify("Failed to return to launch"), 500
 
@app.route('/api/get-mission', methods=['GET'])
def get_mission_data():
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(connect_drone())
        mission = loop.run_until_complete(get_mission(drone))
        mission_dict = mission_plan_to_dict(mission)
        print(f"Mission data to be returned: {mission_dict}")  # Log the mission data being returned
        return jsonify(mission_dict), 200
    except Exception as e:
        print(f"Failed to get mission: {e}")
        return jsonify("Failed to get mission"), 500
 
 
@app.route('/api/upload-mission', methods=['POST'])
def upload_mission_data():
    mission_data = request.json
    try:
        mission_plan = dict_to_mission_plan(mission_data)
        print(f"Mission data received: {mission_data}")

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(connect_drone())
        loop.run_until_complete(upload_mission(drone, mission_plan))

        print("Mission uploaded. Notifying QGroundControl...")
        loop.run_until_complete(notify_qgc_about_mission(drone))

        return jsonify("Mission uploaded and notified to QGroundControl"), 201
    except Exception as e:
        print(f"Failed to upload mission: {e}")
        return jsonify({"error": "Failed to upload mission"}), 400
        
@app.route('/api/start-mission', methods=['POST'])
def start_mission():
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        loop.run_until_complete(connect_drone())

        print("Arming drone...")
        loop.run_until_complete(drone.action.arm())

        print("Taking off...")
        loop.run_until_complete(drone.action.takeoff())

        print("Starting mission...")
        loop.run_until_complete(drone.mission.start_mission())

        print("Mission started successfully.")
        return jsonify({"message": "Mission started successfully"}), 200
    except Exception as e:
        print(f"Failed to start mission: {e}")
        return jsonify({"error": f"Failed to start mission: {e}"}), 500

@app.route('/api/pause-mission', methods=['POST'])
def pause_mission():
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        loop.run_until_complete(connect_drone())

        print("Pausing mission...")
        loop.run_until_complete(drone.mission.pause_mission())

        print("Mission paused successfully.")
        return jsonify({"message": "Mission paused successfully"}), 200
    except Exception as e:
        print(f"Failed to pause mission: {e}")
        return jsonify({"error": f"Failed to pause mission: {e}"}), 500


@app.route('/api/resume-mission', methods=['POST'])
def resume_mission():
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        loop.run_until_complete(connect_drone())

        print("Resuming mission...")
        loop.run_until_complete(drone.mission.start_mission())

        print("Mission resumed successfully.")
        return jsonify({"message": "Mission resumed successfully"}), 200
    except Exception as e:
        print(f"Failed to resume mission: {e}")
        return jsonify({"error": f"Failed to resume mission: {e}"}), 500
    
@app.route('/api/clear-mission', methods=['DELETE'])
def clear_mission():
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        loop.run_until_complete(connect_drone())

        print("Clearing mission...")
        loop.run_until_complete(drone.mission.clear_mission())

        print("Mission cleared successfully.")
        return jsonify({"message": "Mission cleared successfully"}), 200
    except Exception as e:
        print(f"Failed to clear mission: {e}")
        return jsonify({"error": f"Failed to clear mission: {e}"}), 500
 
if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
