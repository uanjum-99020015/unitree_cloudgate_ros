#!/usr/bin/env python3
"""
Unitree G1 movement node: perform gestures and body commands via G1Modes service.
Service: /g1_unit_001/hardware_modes, type: g1_interface/srv/G1Modes.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from unitree_g1_app.g1_services import call_g1_modes

# Default namespace and service (match Unitree G1 docs)
DEFAULT_MODES_SERVICE = "/g1_unit_001/hardware_modes"

# Known gesture/command strings from G1 docs
GESTURES = {
    "damp": "damp",
    "start": "start",
    "squat": "squat",
    "sit": "sit",
    "stand_up": "stand_up",
    "zero_torque": "zero_torque",
    "stop_move": "stop_move",
    "high_stand": "high_stand",
    "low_stand": "low_stand",
    "balance_stand": "balance_stand",
    "shake_hand": "shake_hand",
    "wave_hand": "wave_hand",
    "wave_hand_with_turn": "wave_hand_with_turn",
}


class MovementNode(Node):
    def __init__(self):
        super().__init__("movement_node")
        self.declare_parameter("modes_service", DEFAULT_MODES_SERVICE)
        self.declare_parameter("robot_namespace", "g1_unit_001")
        self.modes_service = self.get_parameter("modes_service").value

        self.gesture_sub = self.create_subscription(
            String,
            "gesture_command",
            self.on_gesture,
            10,
        )
        self.get_logger().info(
            f"Movement node ready. Subscribed to gesture_command. "
            f"Modes service: {self.modes_service}"
        )
        self.get_logger().info(f"Available gestures: {list(GESTURES.keys())}")

    def on_gesture(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        if not cmd:
            return
        request_data = GESTURES.get(cmd, cmd)
        ok, resp = call_g1_modes(self, self.modes_service, request_data)
        if ok:
            self.get_logger().info(f"Gesture '{cmd}' sent. Response: {resp}")
        else:
            self.get_logger().error(f"Gesture '{cmd}' failed.")

    def run_gesture(self, gesture_name: str) -> bool:
        """Call from code: run_gesture('shake_hand')."""
        request_data = GESTURES.get(gesture_name, gesture_name)
        ok, _ = call_g1_modes(self, self.modes_service, request_data)
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
