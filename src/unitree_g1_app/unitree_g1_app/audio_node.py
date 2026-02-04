#!/usr/bin/env python3
"""
Unitree G1 audio node: TTS (speak sentences) and optional audio capture for transcription.
- Speak: calls /g1_unit_001/hardware/audio G1Modes with request_data like 'speak=Hello'
- Audio for transcription: subscribes to an audio topic if available, or records from mic.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from unitree_g1_app.g1_services import call_g1_modes

DEFAULT_AUDIO_SERVICE = "/g1_unit_001/hardware/audio"


class AudioNode(Node):
    def __init__(self):
        super().__init__("audio_node")
        self.declare_parameter("audio_service", DEFAULT_AUDIO_SERVICE)
        self.declare_parameter("default_volume", 80)
        self.audio_service = self.get_parameter("audio_service").value
        self.default_volume = self.get_parameter("default_volume").value

        self.speak_sub = self.create_subscription(
            String,
            "speak",
            self.on_speak,
            10,
        )
        self.transcription_pub = self.create_publisher(String, "transcription", 10)

        self.get_logger().info(
            f"Audio node ready. Subscribed to 'speak'. "
            f"Publishing transcriptions on 'transcription'. Service: {self.audio_service}"
        )

    def on_speak(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return
        self.speak(text)

    def speak(self, text: str, volume: int | None = None) -> bool:
        """Make the robot speak a sentence. volume 0-100."""
        vol = volume if volume is not None else self.default_volume
        request_data = f"volume={vol};speak={text}"
        ok, resp = call_g1_modes(self, self.audio_service, request_data)
        if ok:
            self.get_logger().info(f"Speak: '{text[:50]}...' (vol={vol})")
        else:
            self.get_logger().error("Speak request failed.")
        return ok

    def set_volume(self, volume: int) -> bool:
        """Set speaker volume 0-100."""
        ok, _ = call_g1_modes(self, self.audio_service, f"volume={volume}")
        return ok

    def publish_transcription(self, text: str):
        """Publish transcribed text (e.g. from microphone or G1 audio topic)."""
        msg = String()
        msg.data = text
        self.transcription_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
