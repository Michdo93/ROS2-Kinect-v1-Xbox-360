"""kinect_v1_node – publishes every stream of a Kinect for Xbox 360 as ROS 2 topics.

Topics (below the node namespace, default /kinect_v1)
    rgb/image_raw     sensor_msgs/Image        rgb8 640x480        (video_mode = rgb)
    ir/image_raw      sensor_msgs/Image        mono8 640x480       (video_mode = ir)
    rgb/camera_info, ir/camera_info            sensor_msgs/CameraInfo
    depth/image_raw   sensor_msgs/Image        16UC1, millimetres, registered to RGB
    depth/camera_info sensor_msgs/CameraInfo
    depth/points      sensor_msgs/PointCloud2  xyz (+rgb), thinned out by cloud_step
    accel             sensor_msgs/Imu          accelerometer (linear_acceleration only)
    tilt              std_msgs/Float64         motor angle in degrees
    audio             std_msgs/Int16MultiArray 16 kHz mono PCM blocks (optional, ALSA)

Parameters that can be changed while running (ros2 param set /kinect_v1 ...)
    video_mode  rgb | ir    the Kinect v1 delivers RGB *or* IR, never both at once
    tilt_deg    -27 ... 27  motor
    led         off | green | red | yellow | blink_green | blink_red_yellow

simulate:=true produces a synthetic scene with the same topics – no Kinect needed.
"""

import queue
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from std_msgs.msg import Float64, Int16MultiArray, MultiArrayDimension
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .geometry import camera_info_fields, image_fields, points_xyz, raw_to_mm
from .simulator import Simulator

LEDS = ["off", "green", "red", "yellow", "blink_green", "blink_red_yellow"]


class KinectV1Node(Node):
    def __init__(self):
        super().__init__("driver")
        dp = self.declare_parameter
        self.simulate = dp("simulate", False).value
        self.device_index = dp("device_index", 0).value
        self.frame_prefix = dp("frame_prefix", "kinect_v1").value
        self.fps = float(dp("fps", 30.0).value)
        self.cloud_step = int(dp("cloud_step", 4).value)
        self.cloud_every = int(dp("cloud_every", 3).value)
        self.cloud_color = dp("cloud_color", True).value
        self.audio_enabled = dp("audio", False).value
        self.audio_device = dp("audio_device", "").value
        reliable = dp("reliable", True).value
        self.video_mode = dp("video_mode", "rgb").value
        self.tilt_deg = float(dp("tilt_deg", 0.0).value)
        self.led = dp("led", "green").value
        self.add_on_set_parameters_callback(self._on_parameters)

        qos = QoSProfile(depth=2) if reliable else qos_profile_sensor_data
        pub = lambda t, typ: self.create_publisher(typ, t, qos)  # noqa: E731
        self.pub_rgb, self.pub_ir = pub("rgb/image_raw", Image), pub("ir/image_raw", Image)
        self.pub_depth = pub("depth/image_raw", Image)
        self.pub_cloud = pub("depth/points", PointCloud2)
        self.pub_rgb_info = pub("rgb/camera_info", CameraInfo)
        self.pub_ir_info = pub("ir/camera_info", CameraInfo)
        self.pub_depth_info = pub("depth/camera_info", CameraInfo)
        self.pub_accel = self.create_publisher(Imu, "accel", 10)
        self.pub_tilt = self.create_publisher(Float64, "tilt", 10)
        self.pub_audio = self.create_publisher(Int16MultiArray, "audio", 10)

        self.optical = f"{self.frame_prefix}_rgb_optical_frame"
        self._static_tf()
        self.pending = queue.Queue()  # parameter changes for the capture thread
        self.last_rgb = None  # for the coloured point cloud
        self.frames = 0
        self.stats = {"video": 0, "depth": 0, "audio": 0}
        self.running = True
        self.audio_stream = None

        if self.simulate:
            self.sim = Simulator()
            self.t0 = time.monotonic()
            self.create_timer(1.0 / self.fps, self._simulate_step)
            self.create_timer(0.1, self._simulate_audio)
        else:
            self.capture = threading.Thread(target=self._run_freenect, daemon=True)
            self.capture.start()
            if self.audio_enabled:
                self._start_audio()
        self.create_timer(5.0, self._log_stats)
        self.get_logger().info(
            f"Kinect v1 {'(simulated)' if self.simulate else f'#{self.device_index}'}: "
            f"video {self.video_mode}, depth registered, "
            f"point cloud every {self.cloud_every}. frame"
        )

    # ------------------------------------------------------------------ parameters
    def _on_parameters(self, params):
        for p in params:
            if p.name == "video_mode" and p.value not in ("rgb", "ir"):
                return SetParametersResult(successful=False, reason="video_mode: rgb or ir")
            if p.name == "tilt_deg" and not -27.0 <= float(p.value) <= 27.0:
                return SetParametersResult(successful=False, reason="tilt_deg: -27 ... 27")
            if p.name == "led" and p.value not in LEDS:
                return SetParametersResult(
                    successful=False, reason=f"led: {', '.join(LEDS)}"
                )
        for p in params:
            if p.name in ("video_mode", "tilt_deg", "led"):
                setattr(self, p.name, p.value)
                self.pending.put((p.name, p.value))
                self.get_logger().info(f"{p.name} -> {p.value}")
        return SetParametersResult(successful=True)

    def _static_tf(self):
        """<prefix>_link (x forward, z up) -> <prefix>_rgb_optical_frame (z forward)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f"{self.frame_prefix}_link"
        t.child_frame_id = self.optical
        t.transform.rotation.x, t.transform.rotation.y = -0.5, 0.5
        t.transform.rotation.z, t.transform.rotation.w = -0.5, 0.5
        self.tf = StaticTransformBroadcaster(self)
        self.tf.sendTransform(t)

    # ------------------------------------------------------------------ publishing
    def _header(self, msg, stamp):
        msg.header.stamp = stamp
        msg.header.frame_id = self.optical
        return msg

    def publish_video(self, frame):
        stamp = self.get_clock().now().to_msg()
        info = self._header(CameraInfo(**camera_info_fields()), stamp)
        if self.video_mode == "ir":
            self.pub_ir.publish(self._header(Image(**image_fields(frame, "mono8")), stamp))
            self.pub_ir_info.publish(info)
        else:
            self.last_rgb = frame
            self.pub_rgb.publish(self._header(Image(**image_fields(frame, "rgb8")), stamp))
            self.pub_rgb_info.publish(info)
        self.stats["video"] += 1

    def publish_depth(self, depth_mm):
        stamp = self.get_clock().now().to_msg()
        self.pub_depth.publish(
            self._header(Image(**image_fields(depth_mm, "16UC1")), stamp)
        )
        self.pub_depth_info.publish(self._header(CameraInfo(**camera_info_fields()), stamp))
        self.stats["depth"] += 1
        self.frames += 1
        if (
            self.cloud_every > 0
            and self.frames % self.cloud_every == 0
            and self.pub_cloud.get_subscription_count() > 0
        ):  # only when someone listens
            rgb = self.last_rgb if self.cloud_color and self.video_mode == "rgb" else None
            self.pub_cloud.publish(
                self._cloud(points_xyz(depth_mm, self.cloud_step, rgb=rgb), stamp)
            )

    def _cloud(self, pts, stamp):
        fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate("xyz")
        ]
        if pts.shape[1] == 4:
            fields.append(
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1)
            )
        msg = PointCloud2(
            height=1,
            width=len(pts),
            fields=fields,
            is_bigendian=False,
            point_step=4 * pts.shape[1],
            row_step=4 * pts.shape[1] * len(pts),
            is_dense=True,
            data=np.ascontiguousarray(pts).tobytes(),
        )
        return self._header(msg, stamp)

    def publish_motion(self, accel, tilt):
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = f"{self.frame_prefix}_link"
        imu.orientation_covariance[0] = -1.0  # no orientation
        imu.angular_velocity_covariance[0] = -1.0  # no gyroscope
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (
            accel
        )
        self.pub_accel.publish(imu)
        self.pub_tilt.publish(Float64(data=float(tilt)))

    def publish_audio(self, samples, rate=16000):
        msg = Int16MultiArray(data=samples.ravel().tolist())
        msg.layout.dim = [
            MultiArrayDimension(
                label=f"samples@{rate}Hz", size=samples.size, stride=samples.size
            )
        ]
        self.pub_audio.publish(msg)
        self.stats["audio"] += 1

    # ------------------------------------------------------------------ simulation
    def _simulate_step(self):
        while not self.pending.empty():
            name, value = self.pending.get()
            if name == "tilt_deg":
                self.sim.tilt = float(value)
        t = time.monotonic() - self.t0
        self.publish_video(self.sim.ir(t) if self.video_mode == "ir" else self.sim.rgb(t))
        self.publish_depth(self.sim.depth_mm(t))
        if self.frames % int(self.fps) == 0:
            self.publish_motion(self.sim.accel(), self.sim.tilt)

    def _simulate_audio(self):
        if self.audio_enabled:
            self.publish_audio(self.sim.audio(time.monotonic() - self.t0))

    # ------------------------------------------------------------------ hardware
    def _run_freenect(self):
        """libfreenect's runloop owns this thread; 'body' runs between USB events."""
        import freenect

        modes = {"rgb": freenect.VIDEO_RGB, "ir": freenect.VIDEO_IR_8BIT}
        leds = {name: getattr(freenect, "LED_" + name.upper()) for name in LEDS}
        registered = getattr(freenect, "DEPTH_REGISTERED", None)  # older wrappers: raw only
        state = {"init": False, "last_tilt": 0.0}

        def video(_dev, frame, _ts):
            self.publish_video(frame)

        def depth(_dev, frame, _ts):
            # DEPTH_REGISTERED delivers millimetres aligned to the RGB image, otherwise
            # the 11-bit raw disparity is converted (not aligned to the RGB image)
            self.publish_depth(
                frame.astype(np.uint16) if registered is not None else raw_to_mm(frame)
            )

        def body(dev, _ctx):
            if not self.running or not rclpy.ok():
                raise freenect.Kill
            if not state["init"]:
                if registered is not None:
                    freenect.stop_depth(dev)
                    freenect.set_depth_mode(dev, freenect.RESOLUTION_MEDIUM, registered)
                    freenect.start_depth(dev)
                self.pending.put(("video_mode", self.video_mode))
                self.pending.put(("tilt_deg", self.tilt_deg))
                self.pending.put(("led", self.led))
                state["init"] = True
            while not self.pending.empty():
                name, value = self.pending.get()
                if name == "video_mode":
                    freenect.stop_video(dev)
                    freenect.set_video_mode(dev, freenect.RESOLUTION_MEDIUM, modes[value])
                    freenect.start_video(dev)
                elif name == "tilt_deg":
                    freenect.set_tilt_degs(dev, float(value))
                elif name == "led":
                    freenect.set_led(dev, leds[value])
            now = time.monotonic()
            if now - state["last_tilt"] > 1.0:  # accelerometer and motor once a second
                freenect.update_tilt_state(dev)
                ts = freenect.get_tilt_state(dev)
                self.publish_motion(freenect.get_mks_accel(ts), freenect.get_tilt_degs(ts))
                state["last_tilt"] = now

        try:
            dev = (
                None
                if self.device_index == 0
                else freenect.open_device(freenect.init(), self.device_index)
            )
            freenect.runloop(depth=depth, video=video, body=body, dev=dev)
        except Exception as e:  # no device, USB error ...
            if self.running:
                self.get_logger().error(f"libfreenect: {e}")

    def _start_audio(self):
        """The Kinect microphones appear as ALSA device once the audio firmware is loaded
        (libfreenect fwfetcher.py / kinect_upload_fw)."""
        try:
            import sounddevice as sd

            device = (
                int(self.audio_device)
                if self.audio_device.isdigit()
                else (self.audio_device or None)
            )
            self.audio_stream = sd.InputStream(
                device=device,
                samplerate=16000,
                channels=1,
                dtype="int16",
                blocksize=1600,
                callback=lambda data, *_: self.publish_audio(data.copy()),
            )
            self.audio_stream.start()
        except Exception as e:
            self.get_logger().warn(f"audio disabled: {e}")

    def _log_stats(self):
        self.get_logger().debug(f"frames in 5 s: {self.stats}")
        self.stats = {k: 0 for k in self.stats}

    def destroy_node(self):
        self.running = False
        if self.audio_stream is not None:
            self.audio_stream.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinectV1Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
