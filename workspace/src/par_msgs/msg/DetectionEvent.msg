builtin_interfaces/Time stamp

# Source: "qr", "traffic", "reactive", "gesture"
string source

# Detector payload (QR content, gesture name, colour, etc.).
string payload

# Confidence of the raw detector in [0, 1].
float32 confidence

# Optional 2D / 3D localisation; NaN when unavailable.
float32 image_x
float32 image_y
float32 distance_m
float32 bearing_rad
