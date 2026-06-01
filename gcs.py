from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# -------------------------------
# CONNECTIONS
# -------------------------------
print("Connecting to SCAN vehicle...")
scan = connect('udp:127.0.0.1:14550', wait_ready=True, timeout=60)

print("Connecting to SPRAY vehicle...")
spray = connect('udp:127.0.0.1:14552', wait_ready=True, timeout=60)

# -------------------------------
# STORAGE
# -------------------------------
lat_array = []
lon_array = []
seq_array = []

MAX_WP = 20

prev_seq = None

spray_started = False
current_wp_index = 0

SPRAY_ALT = 3.0

# -------------------------------
# DWELL STATE
# -------------------------------
waiting_at_wp = False
wp_wait_start_time = None
WP_DWELL_TIME = 20   # CHANGED: 5 → 20

# -------------------------------
# RTL INJECTION STATE
# -------------------------------
scan_rtl_injected = False

# -------------------------------
# SERVO HELPER (ADDED ONLY)
# -------------------------------
def set_servo(vehicle, servo_number, pwm):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_number,
        pwm,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# -------------------------------
# HELPER FUNCTIONS
# -------------------------------
def arm_and_takeoff(vehicle, target_alt):
    print("Arming SPRAY vehicle...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {target_alt} meters...")
    vehicle.simple_takeoff(target_alt)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Current altitude: {alt:.2f}")
        if alt >= target_alt * 0.95:
            print(" Target altitude reached")
            break
        time.sleep(1)

def goto_wp(vehicle, lat, lon, alt):
    wp = LocationGlobalRelative(lat, lon, alt)
    print(f" SPRAY going to WP: lat={lat}, lon={lon}, alt={alt}")
    vehicle.simple_goto(wp)

# -------------------------------
# MAIN LOOP
# -------------------------------
print("Starting main loop...")

while True:

    lat = scan.parameters.get('SCR_USER1')
    lon = scan.parameters.get('SCR_USER2')
    seq = scan.parameters.get('SCR_USER3')

    # -------------------------------
    # SCAN ALTITUDE RTL INJECTION
    # -------------------------------
    scan_alt = scan.location.global_relative_frame.alt
    if scan_alt >= 6.5 and not scan_rtl_injected:
        print(f"\nSCAN altitude {scan_alt:.2f}m ≤ 4m — injecting final waypoint for SPRAY")
        lat_array.append(0)
        lon_array.append(0)
        seq_array.append(99)
        scan_rtl_injected = True

    if lat is None or lon is None or seq is None:
        time.sleep(0.2)
        continue

    if prev_seq is None:
        prev_seq = seq
        time.sleep(0.2)
        continue

    # -------------------------------
    # APPEND WAYPOINT ON SEQ CHANGE
    # -------------------------------
    if seq != prev_seq:
        print(f"\nNew waypoint detected (SEQ changed): {prev_seq} → {seq}")

        if len(lat_array) < MAX_WP:
            lat_array.append(lat)
            lon_array.append(lon)
            seq_array.append(seq)

        if len(lat_array) == 4 and not spray_started:
            arm_and_takeoff(spray, SPRAY_ALT)
            goto_wp(spray, lat_array[0], lon_array[0], SPRAY_ALT)
            spray_started = True
            current_wp_index = 0

        prev_seq = seq

    # -------------------------------
    # ARRIVAL DETECTION
    # -------------------------------
    if spray_started and current_wp_index < len(lat_array):

        loc = spray.location.global_relative_frame
        target_lat = lat_array[current_wp_index]
        target_lon = lon_array[current_wp_index]

        dist = ((loc.lat - target_lat)**2 + (loc.lon - target_lon)**2)**0.5

        if dist < 0.000005 and not waiting_at_wp:
            waiting_at_wp = True
            wp_wait_start_time = time.time()

    # -------------------------------
    # DWELL COMPLETE HANDLING (ONLY ADDITIVE)
    # -------------------------------
    if waiting_at_wp:

        elapsed = time.time() - wp_wait_start_time

        if 5 <= elapsed < 15:
            set_servo(spray, 9, 1951)
            print("servo on")   # SERVO ON
        else:
            set_servo(spray, 9, 1051)
            print("servo off")   # SERVO OFF

        if elapsed >= WP_DWELL_TIME:
            waiting_at_wp = False

            if lat_array[current_wp_index] == 0 and lon_array[current_wp_index] == 0:
                spray.mode = VehicleMode("RTL")
            else:
                current_wp_index += 1
                if current_wp_index < len(lat_array):
                    goto_wp(
                        spray,
                        lat_array[current_wp_index],
                        lon_array[current_wp_index],
                        SPRAY_ALT
                    )

    # -------------------------------
    # IDLE DISPATCH (UNCHANGED)
    # -------------------------------
    if (spray_started and
        not waiting_at_wp and
        current_wp_index < len(lat_array) and
        spray.mode.name == "GUIDED"):

        if lat_array[current_wp_index] == 0 and lon_array[current_wp_index] == 0:
            spray.mode = VehicleMode("RTL")
        else:
            goto_wp(
                spray,
                lat_array[current_wp_index],
                lon_array[current_wp_index],
                SPRAY_ALT
            )

    time.sleep(0.2)
