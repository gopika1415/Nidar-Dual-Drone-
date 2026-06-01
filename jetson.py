import cv2
import math
import subprocess
import time
from ultralytics import YOLO
from pymavlink import mavutil

from scaling import gps_scale

# =============================
# CONFIGURATION
# =============================
YOLO_MODEL_PATH = "/home/dracarys/yolo11n_ncnn_model"

CAMERA_INDEX = 4
FRAME_W = 640
FRAME_H = 480
FPS = 30
CONF_THRESH = 0.5

WINDOWS_IP = "10.253.14.174"
STREAM_PORT = 5600

REQUIRED_FRAMES = 5
DIST_ASSOC = 120
DIST_SUPPRESS = 150

PIXHAWK_PORT = "/dev/ttyTHS1"
PIXHAWK_BAUD = 115200

# =============================
# CONNECT TO PIXHAWK (PYMAVLINK)
# =============================
print("🔌 Connecting to Pixhawk using pymavlink...")
master = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
master.wait_heartbeat()
print("✅ Heartbeat received from Pixhawk")

# =============================
# TELEMETRY CACHE
# =============================
telemetry = {
    "lat": None,
    "lon": None,
    "alt": None,
    "roll": None,
    "pitch": None,
    "yaw": None
}

# =============================
# MAVLINK TELEMETRY UPDATE
# =============================
def update_telemetry():
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break

        msg_type = msg.get_type()

        if msg_type == "GLOBAL_POSITION_INT":
            telemetry["lat"] = msg.lat / 1e7
            telemetry["lon"] = msg.lon / 1e7
            telemetry["alt"] = msg.relative_alt / 1000.0

        elif msg_type == "ATTITUDE":
            telemetry["roll"]  = msg.roll
            telemetry["pitch"] = msg.pitch
            telemetry["yaw"]   = msg.yaw

# =============================
# MAVLINK PARAM WRITE FUNCTION
# =============================
def set_param(name, value):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("utf-8"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

# =============================
# YOLO SETUP (ML PART – NOT EXPLAINED)
# =============================
model = YOLO(YOLO_MODEL_PATH, task="detect")
labels = model.names
TARGET_CLASS_ID = [k for k, v in labels.items() if v == "person"][0]

# =============================
# CAMERA SETUP
# =============================
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cap.set(cv2.CAP_PROP_FPS, FPS)

# =============================
# GSTREAMER STREAM
# =============================
def start_gstreamer():
    pipeline = (
        "gst-launch-1.0 fdsrc ! "
        f"videoparse format=bgr width={FRAME_W} height={FRAME_H} framerate={FPS}/1 ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate=1500 speed-preset=superfast ! "
        "rtph264pay pt=96 ! "
        f"udpsink host={WINDOWS_IP} port={STREAM_PORT}"
    )
    return subprocess.Popen(pipeline, shell=True, stdin=subprocess.PIPE)

gst = start_gstreamer()

# =============================
# TRACKING STATE
# =============================
tracks = []
locked_pixels = []
seq_id = 0

def near_locked(cx, cy):
    for lx, ly in locked_pixels:
        if math.hypot(cx - lx, cy - ly) < DIST_SUPPRESS:
            return True
    return False

# =============================
# MAIN LOOP
# =============================
print("🚀 Scan Detection Started (pymavlink)")

while True:
    try:
        update_telemetry()

        if telemetry["alt"] is None or telemetry["alt"] > 4.0:
            time.sleep(0.1)
            continue

        ret, frame = cap.read()
        if not ret:
            continue

        results = model(frame, conf=CONF_THRESH, verbose=False)
        detections = []

        for r in results:
            if r.boxes is None:
                continue

            for box, cls in zip(r.boxes.xyxy, r.boxes.cls):
                if int(cls) != TARGET_CLASS_ID:
                    continue

                x1, y1, x2, y2 = map(int, box)
                cx, cy = (x1 + x2)//2, (y1 + y2)//2

                if near_locked(cx, cy):
                    continue

                detections.append((cx, cy))
                cv2.rectangle(frame, (x1,y1),(x2,y2),(0,255,0),2)

        for cx, cy in detections:
            for tr in tracks:
                if tr["locked"]:
                    continue

                if math.hypot(cx - tr["cx"], cy - tr["cy"]) < DIST_ASSOC:
                    tr["buf"].append((cx, cy))
                    tr["cx"], tr["cy"] = cx, cy

                    if len(tr["buf"]) >= REQUIRED_FRAMES:
                        avg_x = int(sum(p[0] for p in tr["buf"]) / len(tr["buf"]))
                        avg_y = int(sum(p[1] for p in tr["buf"]) / len(tr["buf"]))

                        tr["locked"] = True
                        locked_pixels.append((avg_x, avg_y))
                        seq_id += 1

                        result = gps_scale(
                            avg_x, avg_y,
                            telemetry["lat"],
                            telemetry["lon"],
                            telemetry["alt"],
                            telemetry["yaw"],
                            telemetry["pitch"],
                            telemetry["roll"]
                        )

                        if result:
                            obj_lat, obj_lon = result
                            set_param("SCR_USER1", obj_lat)
                            set_param("SCR_USER2", obj_lon)
                            set_param("SCR_USER3", seq_id)
                            print(f"🎯 ID {seq_id}: {obj_lat}, {obj_lon}")

                    break
            else:
                tracks.append({"cx":cx,"cy":cy,"buf":[(cx,cy)],"locked":False})

        gst.stdin.write(frame.tobytes())

    except KeyboardInterrupt:
        break

cap.release()
gst.terminate()
