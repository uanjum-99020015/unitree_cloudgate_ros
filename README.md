# Unitree G1 ROS2 Jazzy Application

ROS2 Jazzy workspace and package for the **Unitree G1** humanoid robot. It provides:

- **Robot movement** – Gestures and body commands (shake_hand, wave_hand, stand_up, sit, etc.)
- **Audio** – Make the robot speak (TTS) and optional pipeline for audio → transcription
- **Video** – Subscribe to camera images for processing (e.g. card scanning, OCR)

## Requirements

- **ROS2 Jazzy** (Ubuntu 24.04)
- Unitree G1 with **G1 ROS2 stack** installed (e.g. `g1_platform`, `g1_interface`) on the robot or on a PC connected to the robot
- Network: connect to G1 (e.g. Ethernet 192.168.123.164, or set your PC to 192.168.123.51)
- Set **ROS_DOMAIN_ID=10** when running (G1 default):

  ```bash
  export ROS_DOMAIN_ID=10
  ```

Optional for video processing:

- `ros-jazzy-cv-bridge` and OpenCV for image handling

## Workspace layout

```
ros/
├── src/
│   └── unitree_g1_app/    # Main package
│       ├── unitree_g1_app/
│       │   ├── movement_node.py   # Gestures / body commands
│       │   ├── audio_node.py      # Speak + transcription hook
│       │   ├── video_node.py      # Camera image processing
│       │   ├── g1_demo.py         # Short demo sequence
│       │   └── g1_services.py     # G1Modes service helper
│       ├── launch/
│       ├── config/
│       └── package.xml
└── README.md
```

## Build

```bash
cd /path/to/ros
colcon build --symlink-install
source install/setup.bash
```

## Running the G1 stack (on robot or connected PC)

Start the G1 high-level driver and optional modules:

```bash
# Core driver (legs, arms, IMU, etc.)
ros2 launch g1_platform highlevel_ros.launch.py

# Optional: audio (for speak)
ros2 launch g1_platform audio.launch.py

# Optional: video stream / cameras
ros2 launch g1_platform videostream.launch.py
# For RealSense D435i depth + color:
# ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
```

## Running this application

**1. Launch all app nodes (movement, audio, video):**

```bash
export ROS_DOMAIN_ID=10
ros2 launch unitree_g1_app g1_app_launch.py
```

**2. Or run nodes individually:**

```bash
export ROS_DOMAIN_ID=10

# Movement only
ros2 run unitree_g1_app movement_node

# Audio only (speak)
ros2 run unitree_g1_app audio_node

# Video only (camera processing)
ros2 run unitree_g1_app video_node

# One-shot demo (gesture + speak)
ros2 run unitree_g1_app g1_demo
```

**3. Trigger from command line:**

```bash
# Gesture (e.g. wave hand)
ros2 topic pub --once /gesture_command std_msgs/msg/String "{data: 'wave_hand'}"

# Speak
ros2 topic pub --once /speak std_msgs/msg/String "{data: 'Hello, I am G1'}"
```

## Movement (gestures and body commands)

- **Topic:** `gesture_command` (`std_msgs/String`)
- **Service used:** `/g1_unit_001/hardware_modes` (`g1_interface/srv/G1Modes`)

Supported gesture/command names (send as `data` on `gesture_command`):

| Command         | Description                    |
|----------------|--------------------------------|
| `start`        | Start locomotion control       |
| `stand_up`     | Stand up                       |
| `squat`        | Squat posture                  |
| `sit`          | Sit down                       |
| `shake_hand`   | Handshake sequence             |
| `wave_hand`    | Wave hand                      |
| `wave_hand_with_turn` | Wave and turn           |
| `stop_move`    | Stop all motion                |
| `high_stand`   | High stand                     |
| `low_stand`    | Low stand                      |
| `balance_stand`| Balance mode stand             |
| `damp`         | Damping mode                   |
| `zero_torque`  | Disable torque                 |

You can also send custom `request_data` strings if your G1 interface supports them (e.g. `set_velocity=0.3 0.0 0.0 2.0`).

## Audio: make the robot speak

- **Topic:** `speak` (`std_msgs/String`) – publish the sentence to speak.
- **Service used:** `/g1_unit_001/hardware/audio` (`g1_interface/srv/G1Modes`)

Examples:

- `volume=80;speak=Hello from G1`
- Publish to `speak`: `ros2 topic pub --once /speak std_msgs/msg/String "{data: 'Welcome'}"`

## Audio: getting data for transcription

The G1 ROS2 docs describe **speak** (TTS) via the audio service; microphone/audio **stream** topics are not always documented. Options:

1. **Unitree SDK** – On the robot, the SDK example `g1_audio_client_example` may provide raw audio; you can bridge that to a ROS2 topic and subscribe in your own node.
2. **Custom topic** – If your G1 stack or another node publishes audio (e.g. `audio/raw` or similar), subscribe in `audio_node` (or a small dedicated node), run your preferred speech-to-text (e.g. Whisper, Vosk, or cloud API), and publish the result to `transcription` (`std_msgs/String`).
3. **External mic** – Use a separate machine with a microphone, run STT there, and publish transcriptions to `transcription` so other nodes can react.

This package’s `audio_node` already publishes on `transcription`; you only need to feed it text from your STT pipeline.

## Video: camera data for processing (e.g. card scanning)

- **Subscription:** configurable image topic (default: `/camera/camera/color/image_raw` for a typical RealSense).
- **Publication:** `video_processing_result` (`std_msgs/String`) – results of your processing (e.g. “card_detected”, OCR text).

Set the topic in `config/g1_app_params.yaml`:

```yaml
unitree_g1_app:
  ros__parameters:
    image_topic: "/camera/camera/color/image_raw"
    process_interval_frames: 5
```

In `video_node.py`, `process_image()` is a placeholder. Replace it with your logic (OpenCV, OCR, card detection, etc.) and keep publishing a string result to `video_processing_result`.

## Parameters

Edit `config/g1_app_params.yaml` or pass a params file:

- **movement:** `modes_service`, `robot_namespace`
- **audio:** `audio_service`, `default_volume`
- **video:** `image_topic`, `process_interval_frames`
- **demo:** `run_on_start`, `gesture`, `speak_text`

## Notes

- Ensure **arms are straight down** when powering on the G1 so ROS2 control works as expected.
- G1 uses **ROS_DOMAIN_ID=10**; set it in your shell or launch env when talking to the robot.
- If `g1_interface` is not installed, movement and audio nodes will report that the G1 service is unavailable; install the Unitree G1 ROS2 stack on the side that calls those services.
