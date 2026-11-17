import numpy as np

from kinect_v1_ros2.geometry import camera_info_fields, image_fields, points_xyz, raw_to_mm
from kinect_v1_ros2.simulator import Simulator


def test_camera_info():
    info = camera_info_fields()
    assert info["k"][0] == 525.0 and info["k"][2] == 319.5 and len(info["p"]) == 12


def test_image_fields():
    img = np.zeros((480, 640, 3), np.uint8)
    f = image_fields(img, "rgb8")
    assert f["step"] == 1920 and len(f["data"]) == 921600
    assert image_fields(np.zeros((480, 640), np.uint16), "16UC1")["step"] == 1280


def test_raw_to_mm():
    mm = raw_to_mm(np.array([[2047, 600, 800]], np.uint16))
    assert mm[0, 0] == 0 and 500 < mm[0, 1] < 700 and 1000 < mm[0, 2] < 1300


def test_points_centre_pixel():
    d = np.zeros((480, 640), np.uint16)
    d[240, 320] = 2000  # pixel next to the principal point, 2 m away
    pts = points_xyz(d, step=1)
    assert pts.shape == (1, 3)
    assert abs(pts[0, 2] - 2.0) < 1e-6 and abs(pts[0, 0]) < 0.01


def test_points_with_colour():
    d = np.full((480, 640), 1000, np.uint16)
    rgb = np.zeros((480, 640, 3), np.uint8)
    rgb[..., 0] = 255
    pts = points_xyz(d, step=4, rgb=rgb)
    assert pts.shape == (160 * 120, 4)
    assert pts[0, 3].view(np.uint32) == 0xFF0000


def test_simulator_hand_every_ten_seconds():
    sim = Simulator()
    assert not sim.person(1.0)[2] and sim.person(8.0)[2]
    near = sim.depth_mm(8.0)
    assert near[40:120].min() > 0 and near[40:120][near[40:120] > 0].min() < 1900
    assert sim.rgb(0).shape == (480, 640, 3) and sim.ir(0).dtype == np.uint8
