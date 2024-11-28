from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import redis
import json

app = FastAPI()

# CORS Middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

redis_client = redis.StrictRedis(host='localhost', port=6379, db=0)

@app.get("/gps")
async def get_gps():
    gps_data_json = redis_client.get("gps_data")
    if gps_data_json:
        gps_data = json.loads(gps_data_json)
        print(f"Redis'ten alınan GPS verileri: {gps_data}")
    else:
        gps_data = {'latitude': None, 'longitude': None, 'altitude': None, 'timestamp': None}
        print("Redis'ten GPS verileri alınamadı.")
    return gps_data
    
@app.get("/velocity")
async def get_velocity():
    velocity_data_json = redis_client.get("velocity_data")
    
    if velocity_data_json:
        velocity_data = json.loads(velocity_data_json)
        return velocity_data
    else:
        return {"error": "Hız verisi bulunamadı"}
