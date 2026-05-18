builtin_interfaces/Time stamp

# "GREEN" | "YELLOW" | "RED" | "UNKNOWN"
string state

# Detector confidence in [0, 1].
float32 confidence

# Distance to the detected signal card in metres, from depth. NaN if unavailable.
float32 distance_m

# Bearing to the signal in radians, relative to camera forward. NaN if unavailable.
float32 bearing_rad
