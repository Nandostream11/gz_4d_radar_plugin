import os
import cv2
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
from .georeference import GeoReference

# PX4 SensorGps message definition for rosbags typestore
PX4_SENSOR_GPS_MSG = """
uint64 timestamp
uint64 timestamp_sample
uint32 device_id
int32 lat
int32 lon
int32 alt
int32 alt_ellipsoid
float32 s_variance_m_s
float32 c_variance_rad
uint8 fix_type
float32 eph
float32 epv
float32 hdop
float32 vdop
int32 noise_per_ms
uint16 automatic_gain_control
uint8 jamming_state
int32 vel_m_s
int32 vel_n_m_s
int32 vel_e_m_s
int32 vel_d_m_s
bool vel_ned_valid
float32 cog_rad
int32 timestamp_time_relative
float32 heading
float32 heading_offset
float32 heading_accuracy
float32 rtcm_injection_rate
uint8 selected_rtcm_instance
"""

def get_configured_typestore():
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    try:
        from rosbags.typesys.msg import generate_msgdef
        # Register px4_msgs/msg/SensorGps if needed
        typestore.register(
            typestore.FIELD_DEFS.get('px4_msgs/msg/SensorGps', {}),
            'px4_msgs/msg/SensorGps'
        )
    except Exception:
        pass
    return typestore

class BagParser:
    """
    Extracts time-synchronized camera frames, GPS trajectory, and IMU orientations
    from a ROS 2 MCAP or DB3 bag file.
    """
    def __init__(self, bag_path: str):
        self.bag_path = bag_path
        self.typestore = get_configured_typestore()

    def parse(self, min_spatial_step_m: float = 0.5):
        gps_fixes = []   # (timestamp_ns, lat, lon, alt)
        imu_records = [] # (timestamp_ns, qx, qy, qz, qw)
        images = []      # (timestamp_ns, cv2_bgr_img)

        with Reader(self.bag_path) as reader:
            for connection, timestamp, rawdata in reader.messages():
                topic = connection.topic
                msgtype = connection.msgtype

                # 1. Parse GPS Fixes (/fmu/in/sensorgps or /gps/fix)
                if topic in ["/fmu/in/sensorgps", "/fmu/out/vehicle_gps_position", "/gps/fix"]:
                    try:
                        msg = self.typestore.deserialize_cdr(rawdata, msgtype)
                        lat = getattr(msg, 'lat', getattr(msg, 'latitude', 0.0))
                        lon = getattr(msg, 'lon', getattr(msg, 'longitude', 0.0))
                        alt = getattr(msg, 'alt', getattr(msg, 'altitude', 0.0))

                        # Scale integer GPS units from PX4 if in 1e7 degrees
                        if abs(lat) > 1e4: lat *= 1e-7
                        if abs(lon) > 1e4: lon *= 1e-7
                        if abs(alt) > 1e4: alt *= 1e-3

                        gps_fixes.append((timestamp, float(lat), float(lon), float(alt)))
                    except Exception:
                        pass

                # 2. Parse IMU (/imu/data)
                elif topic in ["/imu/data", "/fmu/out/vehicle_attitude"]:
                    try:
                        msg = self.typestore.deserialize_cdr(rawdata, msgtype)
                        if hasattr(msg, 'orientation'):
                            q = msg.orientation
                            imu_records.append((timestamp, float(q.x), float(q.y), float(q.z), float(q.w)))
                        elif hasattr(msg, 'q'):
                            q = msg.q  # PX4 attitude quaternion [w, x, y, z]
                            imu_records.append((timestamp, float(q[1]), float(q[2]), float(q[3]), float(q[0])))
                    except Exception:
                        pass

                # 3. Parse Camera Images (/camera/image_rect or /camera/image_raw)
                elif topic in ["/camera/image_rect", "/camera/image_raw", "/camera/color/image_raw"]:
                    try:
                        msg = self.typestore.deserialize_cdr(rawdata, msgtype)
                        encoding = getattr(msg, 'encoding', 'bgr8')
                        h, w = msg.height, msg.width
                        raw_buf = np.frombuffer(msg.data, dtype=np.uint8)

                        if 'bgr' in encoding or 'rgb' in encoding:
                            channels = 3
                            img = raw_buf.reshape((h, w, channels))
                            if 'rgb' in encoding:
                                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        elif 'mono8' in encoding:
                            img = raw_buf.reshape((h, w))
                            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                        else:
                            img = raw_buf.reshape((h, w, -1))

                        images.append((timestamp, img))
                    except Exception:
                        pass

        if not gps_fixes:
            raise ValueError(f"No GPS fixes found on topics [/fmu/in/sensorgps, /gps/fix] in {self.bag_path}")

        # Initialize GeoReference with initial GPS fix
        ref_lat, ref_lon, ref_alt = gps_fixes[0][1], gps_fixes[0][2], gps_fixes[0][3]
        georef = GeoReference(ref_lat, ref_lon, ref_alt)

        # Synchronize and decimate keyframes
        keyframes = []
        last_pos_enu = np.array([-1e9, -1e9, -1e9])

        # Pre-convert GPS fixes to ENU
        gps_timestamps = np.array([g[0] for g in gps_fixes])
        gps_enu = np.array([georef.lla_to_enu(g[1], g[2], g[3]) for g in gps_fixes])

        imu_timestamps = np.array([i[0] for i in imu_records]) if imu_records else np.array([])
        imu_quats = np.array([[i[1], i[2], i[3], i[4]] for i in imu_records]) if imu_records else np.array([])

        for t_img, img in images:
            # Match nearest GPS fix
            idx_gps = np.argmin(np.abs(gps_timestamps - t_img))
            pos_enu = gps_enu[idx_gps]

            # Enforce spatial minimum distance between keyframes
            if np.linalg.norm(pos_enu - last_pos_enu) >= min_spatial_step_m:
                # Match nearest IMU orientation
                if len(imu_timestamps) > 0:
                    idx_imu = np.argmin(np.abs(imu_timestamps - t_img))
                    quat = imu_quats[idx_imu]
                else:
                    quat = np.array([0.0, 0.0, 0.0, 1.0])

                keyframes.append({
                    "timestamp": t_img,
                    "image": img,
                    "pos_enu": pos_enu,
                    "quat": quat
                })
                last_pos_enu = pos_enu

        return keyframes, georef
