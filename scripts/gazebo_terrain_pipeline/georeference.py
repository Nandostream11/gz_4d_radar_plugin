import numpy as np

class GeoReference:
    """
    Converts WGS84 Geodetic coordinates (Latitude, Longitude, Altitude)
    to a Local East-North-Up (ENU) Cartesian frame in meters.
    """
    def __init__(self, ref_lat: float, ref_lon: float, ref_alt: float):
        self.ref_lat = np.radians(ref_lat)
        self.ref_lon = np.radians(ref_lon)
        self.ref_alt = ref_alt
        
        # WGS-84 Ellipsoid constants
        self.a = 6378137.0             # Earth semi-major axis (meters)
        self.f = 1.0 / 298.257223563   # Flattening
        self.e2 = 2 * self.f - self.f ** 2
        
        self.ref_ecef = self._lla_to_ecef(self.ref_lat, self.ref_lon, self.ref_alt)

    def _lla_to_ecef(self, lat_rad, lon_rad, alt):
        N = self.a / np.sqrt(1.0 - self.e2 * np.sin(lat_rad)**2)
        x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
        y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
        z = (N * (1.0 - self.e2) + alt) * np.sin(lat_rad)
        return np.array([x, y, z])

    def lla_to_enu(self, lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
        lat = np.radians(lat_deg)
        lon = np.radians(lon_deg)
        ecef = self._lla_to_ecef(lat, lon, alt_m)
        diff = ecef - self.ref_ecef
        
        phi, lam = self.ref_lat, self.ref_lon
        R = np.array([
            [-np.sin(lam),              np.cos(lam),             0],
            [-np.sin(phi)*np.cos(lam), -np.sin(phi)*np.sin(lam), np.cos(phi)],
            [ np.cos(phi)*np.cos(lam),  np.cos(phi)*np.sin(lam), np.sin(phi)]
        ])
        return R @ diff  # [X_East, Y_North, Z_Up]
