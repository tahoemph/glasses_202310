# SPDX-FileCopyrightText: 2021 Phil Burgess for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
This version started out as GOOGLY EYES for Adafruit EyeLight LED glasses + driver. Pendulum physics
That was Adapted from Bill Earl's STEAM-Punk Goggles project: https://learn.adafruit.com/steam-punk-goggles
"""
import math
import random
import time

import board
import supervisor
import adafruit_lis3dh
import adafruit_is31fl3741
from adafruit_is31fl3741.adafruit_ledglasses import LED_Glasses
from adafruit_is31fl3741.adafruit_rgbmatrixqt import Adafruit_RGBMatrixQT

# HARDWARE SETUP ----

# Shared by both the accelerometer and LED controller
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Initialize the accelerometer
lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c)

# Initialize the IS31 LED driver, buffered for smoother animation
glasses = LED_Glasses(i2c, allocate=adafruit_is31fl3741.MUST_BUFFER)
# matrix = Adafruit_RGBMatrixQT(i2c, allocate=adafruit_is31fl3741.MUST_BUFFER)

def make_color(r, g, b, scale=1):
    return (int(r * scale) << 16 | int(g * scale) << 8 | int(b * scale))

def store_color(r, g, b):
    rv = (r, g, b)
    return rv

def move_pixel(pixel, direction):
    pixel = pixel + direction
    if pixel < 0:
        pixel = 23
    if pixel > 23:
        pixel = 0
    return pixel

class Trail:
    def __init__(self, length):
        self.path = []
        self.pixel = 6
        self.ring = glasses.left_ring
        self.direction = 1
        self.length = length

    def step(self):
        # Walk the tail adjusting
        index = 0
        while index < len(self.path):
            ring, pixel, color = self.path[index]
            new_color = store_color(int(color[0]/3), int(color[1]/3), int(color[2]/3))
            ring[pixel] = make_color(*new_color)
            self.path[index] = (ring, pixel, new_color)
            index += 1

        if len(self.path) > self.length:
            ring, pixel, _ = self.path.pop()
            ring[pixel] = make_color(0, 0, 0)

        # Adjust for this state
        if self.ring == glasses.left_ring and self.pixel == 5:
            self.ring = glasses.right_ring
            self.pixel = 18 
            self.direction = -1
        elif self.ring == glasses.right_ring and self.pixel == 19:
            self.ring = glasses.left_ring
            self.pixel = 6
            self.direction = 1
        else:
            self.pixel = move_pixel(self.pixel, self.direction)

        color = store_color(0, 255, 0)
        self.ring[self.pixel] = make_color(*color)
        self.path.insert(0, (self.ring, self.pixel, color))

class Explosion:
    def __init__(self):
        self.foo = 1

    def display(self):
        for pixel in range(10):
            matrix.pixel(5, pixel, make_color(2555, 0, 0))

while True:

    # The try/except here is because VERY INFREQUENTLY the I2C bus will
    # encounter an error when accessing either the accelerometer or the
    # LED driver, whether from bumping around the wires or sometimes an
    # I2C device just gets wedged. To more robustly handle the latter,
    # the code will restart if that happens.
    trail = Trail(4)
    # explosion = Explosion()
    try:
        accel = lis3dh.acceleration
        # things = [(100, trail.step), (1000, explosion.display)]
        things = [(100, trail.step)]
        last_run = [supervisor.ticks_ms()] * len(things)
        while True:
            for index, (freq, operation) in enumerate(things):
                delta = supervisor.ticks_ms() - last_run[index];
                if delta > freq:
                    last_run[index] = supervisor.ticks_ms()
                    operation()
                    glasses.show()

    # See "try" notes above regarding rare I2C errors.
    except OSError:
        supervisor.reload()
