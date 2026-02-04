#!/usr/bin/env python3
"""
Demo node: runs a short sequence of G1 operations (gesture, speak, video processing).
Use as template or run once to verify connectivity.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from unitree_g1_app.g1_services import call_g1_modes

MODES_SERVICE = "/g1_unit_001/hardware_modes"
AUDIO_SERVICE = "/g1_unit_001/hardware/audio"


class G1DemoNode(Node):
    def __init__(self):
        super().__init__("g1_demo")
        self.declare_parameter("run_on_start", True)
        self.declare_parameter("gesture", "wave_hand")
        self.declare_parameter("speak_text", "Hello from Unitree G1 application.")
        if self.get_parameter("run_on_start").value:
            self.timer = self.create_timer(2.0, self.run_once)

    def run_once(self):
        self.timer.cancel()
        gesture = self.get_parameter("gesture").value
        speak_text = self.get_parameter("speak_text").value
        self.get_logger().info("Running demo: gesture then speak.")
        ok, _ = call_g1_modes(self, MODES_SERVICE, gesture)
        self.get_logger().info(f"Gesture '{gesture}': {'ok' if ok else 'failed'}")
        ok, _ = call_g1_modes(self, AUDIO_SERVICE, f"volume=80;speak={speak_text}")
        self.get_logger().info(f"Speak: {'ok' if ok else 'failed'}")


def main(args=None):
    rclpy.init(args=args)
    node = G1DemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
