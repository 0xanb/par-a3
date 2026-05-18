"""Subscribe to /par/events and /par/detections, write per-trial CSV."""
import rclpy
from rclpy.node import Node


class Recorder(Node):
 def __init__(self) -> None:
 super.__init__("recorder")
 self.get_logger.info("par_eval.recorder stub")


def main(args=None) -> None:
 rclpy.init(args=args)
 node = Recorder
 try:
 rclpy.spin(node)
 finally:
 node.destroy_node
 rclpy.shutdown
