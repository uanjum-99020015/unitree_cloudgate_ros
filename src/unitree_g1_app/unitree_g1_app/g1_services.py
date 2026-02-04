"""
Helper to call G1 ROS2 services (G1Modes).
Works with or without g1_interface package; uses dynamic service type when available.
"""
import rclpy
from rclpy.node import Node

# Optional: G1 interface from Unitree G1 ROS2 stack
try:
    from g1_interface.srv import G1Modes
    HAS_G1_INTERFACE = True
except ImportError:
    G1Modes = None
    HAS_G1_INTERFACE = False


def call_g1_modes(node: Node, service_name: str, request_data: str, timeout_sec: float = 10.0):
    """
    Call a G1Modes service (e.g. hardware_modes, audio, led).
    request_data: e.g. "shake_hand", "speak=Hello", "volume=80"
    Returns (success: bool, response_data: str or None).
    """
    if not HAS_G1_INTERFACE:
        node.get_logger().warn(
            "g1_interface not installed. Install Unitree G1 ROS2 stack for real robot."
        )
        return False, None

    client = node.create_client(G1Modes, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().error(f"Service {service_name} not available.")
        return False, None

    req = G1Modes.Request()
    req.request_data = request_data
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        node.get_logger().error(f"Service {service_name} call timed out.")
        return False, None
    result = future.result()
    if result is None:
        return False, None
    return True, getattr(result, "response_data", None)
