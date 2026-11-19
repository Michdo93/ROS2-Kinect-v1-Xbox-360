# ROS2-Kinect-v1-Xbox-360

ROS 2 driver node for the **Kinect for Xbox 360** (Kinect v1, models 1414/1473) based on
[libfreenect](https://github.com/OpenKinect/libfreenect). One node publishes every stream of
the sensor once – any number of applications (viewer, recorder, gesture or presence
detection, RViz) subscribe to what they need.

`simulate:=true` produces a synthetic scene with exactly the same topics (a person walking
around who raises a hand for 3 s every 10 s) – everything can be developed without a Kinect.

Tested with ROS 2 **Jazzy** and **Lyrical** (Ubuntu 24.04/26.04, Raspberry Pi 4/5).

```
                          /kinect_v1/rgb/image_raw     rgb8  640x480 30 Hz  ~28 MB/s
 Kinect v1 ─ libfreenect ─ driver ──► /kinect_v1/ir/image_raw      mono8 (instead of RGB)
                          │       ──► /kinect_v1/depth/image_raw   16UC1 mm, registered ~18 MB/s
                          │       ──► /kinect_v1/depth/points      PointCloud2 xyz+rgb 10 Hz
                          │       ──► /kinect_v1/{rgb,ir,depth}/camera_info
                          │       ──► /kinect_v1/accel  /kinect_v1/tilt  /kinect_v1/audio
                          └ parameters: video_mode (rgb|ir), tilt_deg, led
```

## Hardware notes

* The Kinect v1 needs its **external power supply / USB adapter** – USB power is not enough.
* The sensor delivers **RGB or IR, never both at the same time** (one video channel). Switch
  at runtime: `ros2 param set /kinect_v1/driver video_mode ir`. The IR image shows the
  projected dot pattern and works in complete darkness; depth keeps running in both modes.
* Depth is requested as `DEPTH_REGISTERED` (millimetres, aligned to the RGB image). Older
  Python wrappers without that mode fall back to the 11-bit raw values, converted to mm.
* A Raspberry Pi 4 or 5 handles all streams; a Pi 3 only with reduced rates.

## Installation

```bash
sudo apt install freenect python3-freenect python3-numpy      # libfreenect + Python wrapper
freenect-glview                                               # hardware test
# access without sudo: libfreenect's udev rule 51-kinect.rules to /etc/udev/rules.d/

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Michdo93/ROS2-Kinect-v1-Xbox-360.git
cd .. && source /opt/ros/lyrical/setup.bash                   # or jazzy
colcon build --symlink-install && source install/setup.bash
```

Microphone (optional): the Kinect appears as ALSA device only after its audio firmware has
been uploaded (libfreenect `fwfetcher.py` / `kinect_upload_fw`). Then
`pip install --break-system-packages sounddevice` and `audio: true`.

## Usage

```bash
ros2 launch kinect_v1_ros2 kinect_v1.launch.py                 # hardware
ros2 launch kinect_v1_ros2 kinect_v1.launch.py simulate:=true  # no Kinect needed
ros2 run rqt_image_view rqt_image_view /kinect_v1/rgb/image_raw
ros2 param set /kinect_v1/driver tilt_deg 10.0
ros2 param set /kinect_v1/driver led blink_green
```

Parameters (`config/kinect_v1.yaml`):

| Parameter | Default | Meaning |
|---|---|---|
| `simulate` | false | synthetic scene instead of the hardware |
| `device_index` | 0 | several Kinects on one computer |
| `frame_prefix` | kinect_v1 | frames `<prefix>_link` and `<prefix>_rgb_optical_frame` (static TF) |
| `video_mode` | rgb | `rgb` or `ir`, changeable at runtime |
| `tilt_deg` | 0.0 | motor, −27 … 27, changeable at runtime |
| `led` | green | off, green, red, yellow, blink_green, blink_red_yellow |
| `cloud_step` | 4 | point cloud from every 4th pixel (160 × 120 points) |
| `cloud_every` | 3 | point cloud every 3rd depth frame (only while someone subscribes) |
| `cloud_color` | true | add the RGB colour to every point |
| `reliable` | true | `false`: best-effort QoS (sensor data) – better over Wi-Fi |
| `audio`, `audio_device` | false, "" | microphone via ALSA |
| `fps` | 30.0 | simulation only |

With `reliable: false` subscribers must use best-effort QoS as well
(`qos_profile_sensor_data`); RViz and `rqt_image_view` can switch it in their settings.

Several Kinects: start the launch file once per device, e.g.
`namespace:=kinect_flur device_index:=1` – the frames are named after the namespace
(`kinect_flur_link`, `kinect_flur_rgb_optical_frame`).

## Tests

```bash
python3 -m pytest -q test          # geometry, point cloud, simulator – no ROS, no Kinect
```

GitHub Actions builds the package for Jazzy and Lyrical, runs the tests and starts the
simulation.

## Related

* [ROS2-Kinect-v2-Xbox-One](https://github.com/Michdo93/ROS2-Kinect-v2-Xbox-One) – the same for
  the Kinect v2
* [ros2_openhab_ws](https://github.com/Michdo93/ros2_openhab_ws) – bridge between ROS 2 and
  openHAB
* Book *Smart Home & ROS2* (Praxisreihe Smart Home) by Michael Christian Dörflinger

## License

MIT
