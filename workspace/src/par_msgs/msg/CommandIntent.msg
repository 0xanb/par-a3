# CommandIntent: any of our behaviour nodes emits one of these.
# The arbiter fuses them by priority + confidence into a single /cmd_vel.

builtin_interfaces/Time stamp

# Source identifies which behaviour produced the intent.
# Values: "qr", "traffic", "reactive", "gesture", "safety", "default"
string source

# Priority: higher wins ties. The arbiter also uses a per-source weight.
# Suggested: safety=100, traffic_red=90, reactive=70, qr=60, gesture=50, default=10.
uint8 priority

# Confidence in [0, 1]. Low-confidence intents are ignored by the arbiter.
float32 confidence

# Requested motion.
geometry_msgs/Twist cmd

# Semantic label for logs (e.g. "TURN_LEFT", "RED", "AVOID", "THUMBS_UP").
string label
