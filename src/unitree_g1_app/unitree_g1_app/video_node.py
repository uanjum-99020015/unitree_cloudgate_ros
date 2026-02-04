#!/usr/bin/env python3
"""
Unitree G1 video node: subscribe to camera image topic for processing (e.g. card scanning).
Publishes processed results (e.g. detected card text) on a result topic.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Try cv_bridge for Image <-> numpy (optional)
try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    CvBridge = None
    HAS_CV_BRIDGE = False

# Common G1 / RealSense topic names
DEFAULT_IMAGE_TOPIC = "/camera/camera/color/image_raw"


class VideoNode(Node):
    def __init__(self):
        super().__init__("video_node")
        self.declare_parameter("image_topic", DEFAULT_IMAGE_TOPIC)
        self.declare_parameter("process_interval_frames", 5)
        self.image_topic = self.get_parameter("image_topic").value
        self.process_every_n = self.get_parameter("process_interval_frames").value
        self.frame_count = 0
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            10,
        )
        self.result_pub = self.create_publisher(String, "video_processing_result", 10)

        self.get_logger().info(
            f"Video node ready. Subscribing to '{self.image_topic}'. "
            f"Publishing results on 'video_processing_result'."
        )
        if not HAS_CV_BRIDGE:
            self.get_logger().warn(
                "cv_bridge not found (Ubuntu/ROS2 Jazzy: sudo apt install ros-jazzy-cv-bridge). "
                "Image callback will only log receipt."
            )

    def on_image(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return
        result = self.process_image(msg)
        if result is not None:
            out = String()
            out.data = result
            self.result_pub.publish(out)

    def process_image(self, msg: Image) -> str | None:
        """
        Process one image (e.g. scan card, OCR). Override or replace for real logic.
        Returns a string result to publish, or None to skip.
        """
        if self.bridge is None:
            self.get_logger().info(
                f"Received image {msg.width}x{msg.height} (no cv_bridge, skipping processing)"
            )
            return None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return None
        # Placeholder: you can add OpenCV/OCR here (e.g. card detection, text extraction)
        # Example: return f"card_detected,confidence=0.9"
        return f"frame_{self.frame_count},size={cv_image.shape[1]}x{cv_image.shape[0]}"


def main(args=None):
    rclpy.init(args=args)
    node = VideoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
