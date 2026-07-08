# Active behaviour mode of the robot. Latched on /par/active_mode.
#
# IDLE — boot default; no behaviour is acting; LEDs off.
# A    — Reactive nav + QR-restricted vocabulary (STOP, GO).
# B    — Hand-gesture command (sit-down posture).
#
# Legacy values (preserved for snapshot-pre-2mode rollback):
# C, D — see git tag 'snapshot-pre-2mode' for the 5-scene demo.
#
# For the 2-mode pivot.

builtin_interfaces/Time stamp

# One of: "IDLE" | "A" | "B"  (legacy: "C" | "D" preserved for snapshot rollback)
string mode

# Why this mode is now active. One of:
#   "boot"           — initial publish on system startup
#   "supervisor"     — supervisor FSM transition (button press, validate result)
#   "manual"         — operator forced via scripts/scene.sh
#   "qr_switch"      — operator showed a MODE_X QR card (legacy)
#   "system_revert"  — internal fault returned us to mode A (rare)
string reason
