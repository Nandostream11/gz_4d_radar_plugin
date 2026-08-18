# Standard ROS 2 Radar Return message
# Represents a single 4D radar detection

float32 range               # [m] Radial distance to target
float32 azimuth             # [rad] Azimuth angle in sensor frame (+y left)
float32 elevation           # [rad] Elevation angle in sensor frame (+z up)
float32 doppler_velocity     # [m/s] Radial relative Doppler velocity
float32 amplitude           # [dBm^2] Radar cross section / return amplitude
