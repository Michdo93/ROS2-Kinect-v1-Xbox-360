"""Camera model, ROS message helpers and point cloud – numpy only, unit-tested."""

import numpy as np

WIDTH, HEIGHT = 640, 480
# Nominal intrinsics of the Kinect v1 RGB camera (depth is registered to it).
# Calibrate your own device with camera_calibration for better values.
FX = FY = 525.0
CX, CY = 319.5, 239.5


def camera_info_fields(width=WIDTH, height=HEIGHT, fx=FX, fy=FY, cx=CX, cy=CY):
    """Values for sensor_msgs/CameraInfo (plumb bob, no distortion)."""
    return {
        "width": width,
        "height": height,
        "distortion_model": "plumb_bob",
        "d": [0.0] * 5,
        "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    }


def image_fields(array, encoding):
    """Values for sensor_msgs/Image from a numpy array (rgb8, mono8, 16UC1)."""
    array = np.ascontiguousarray(array)
    height, width = array.shape[:2]
    return {
        "height": height,
        "width": width,
        "encoding": encoding,
        "is_bigendian": 0,
        "step": array.strides[0],
        "data": array.tobytes(),
    }


def raw_to_mm(raw):
    """11-bit raw disparity -> millimetres (OpenKinect approximation), 0 = no value."""
    raw = raw.astype(np.float32)
    with np.errstate(divide="ignore"):
        mm = 1000.0 / (raw * -0.0030711016 + 3.3309495161)
    mm[(raw >= 2047) | (mm <= 0) | (mm > 10000)] = 0
    return mm.astype(np.uint16)


def points_xyz(depth_mm, step=4, fx=FX, fy=FY, cx=CX, cy=CY, rgb=None):
    """Depth image in mm -> N x 3 float32 points in metres (optical frame: z forward).

    step thins out the image (step 4: 160 x 120 = 19 200 points instead of 307 200).
    With rgb (H x W x 3, registered) a packed float32 'rgb' column is added.
    """
    d = depth_mm[::step, ::step].astype(np.float32) / 1000.0
    v, u = np.mgrid[0 : depth_mm.shape[0] : step, 0 : depth_mm.shape[1] : step]
    valid = d > 0
    z = d[valid]
    x = (u[valid] - cx) * z / fx
    y = (v[valid] - cy) * z / fy
    pts = np.column_stack([x, y, z]).astype(np.float32)
    if rgb is None:
        return pts
    c = rgb[::step, ::step][valid].astype(np.uint32)
    packed = ((c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]).view(np.float32)
    return np.column_stack([pts, packed])
