"""Synthetic Kinect v1 scene: wall, a person walking around and raising a hand now and then.

Same formats as the real sensor, so every subscriber can be developed without a Kinect.
"""
import math

import numpy as np

from .geometry import HEIGHT, WIDTH


class Simulator:
    def __init__(self, seed=7):
        self.yy, self.xx = np.mgrid[0:HEIGHT, 0:WIDTH]
        self.rng = np.random.default_rng(seed)
        self.tilt = 0.0

    def person(self, t):
        """Centre column, distance in mm, hand raised?"""
        cx = int(320 + 150 * math.sin(t * 0.5))
        dist = int(1500 + 700 * (1 + math.sin(t * 0.8)) / 2)  # 1.5 ... 2.2 m
        hand = (t % 10.0) > 7.0                                # 3 s of every 10 s
        return cx, dist, hand

    def depth_mm(self, t):
        d = np.full((HEIGHT, WIDTH), 3000, np.uint16)          # wall at 3 m
        d[440:, :] = 0                                         # floor edge: no value
        cx, dist, hand = self.person(t)
        d[140:440, max(cx - 55, 0):cx + 55] = dist             # body
        d[90:140, max(cx - 25, 0):cx + 25] = dist              # head
        if hand:                                               # raised hand, 40 cm in front
            d[40:120, max(cx + 60, 0):cx + 100] = dist - 400
        return d

    def rgb(self, t):
        img = np.stack([self.xx * 255 // (WIDTH - 1), self.yy * 255 // (HEIGHT - 1),
                        np.full_like(self.xx, 128)], axis=-1).astype(np.uint8)
        cx, _, hand = self.person(t)
        img[140:440, max(cx - 55, 0):cx + 55] = (40, 90, 200)
        img[90:140, max(cx - 25, 0):cx + 25] = (230, 190, 160)
        if hand:
            img[40:120, max(cx + 60, 0):cx + 100] = (230, 190, 160)
        return img

    def ir(self, t):
        speckle = self.rng.integers(0, 60, (HEIGHT, WIDTH), dtype=np.uint8)
        base = (255 - self.depth_mm(t) // 15).clip(0, 255).astype(np.uint8)
        return np.maximum(base // 2, speckle)

    def audio(self, t, frames=1600, rate=16000):
        k = np.arange(frames) / rate + t
        return (3000 * np.sin(2 * math.pi * 440 * k)).astype(np.int16)

    def accel(self):
        a = math.radians(self.tilt)
        return 0.0, -9.81 * math.cos(a), 9.81 * math.sin(a)
