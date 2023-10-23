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


# HARDWARE SETUP ----

# Shared by both the accelerometer and LED controller
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Initialize the accelerometer
lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c)

# Initialize the IS31 LED driver, buffered for smoother animation
glasses = LED_Glasses(i2c, allocate=adafruit_is31fl3741.MUST_BUFFER)


# PHYSICS SETUP -----


class Pendulum:
    """A small class for our pendulum simulation."""

    def __init__(self, ring, color):
        """Initial pendulum position, plus axle friction, are randomized
        so the two rings don't spin in perfect lockstep."""
        self.ring = ring  # Save reference to corresponding LED ring
        self.color = color  # (R,G,B) tuple for color
        self.angle = random.random()  # Position around ring, in radians
        self.momentum = 0
        self.friction = random.uniform(0.85, 0.9)  # Inverse friction, really

    def interp(self, pixel, scale):
        """Given a pixel index (0-23) and a scaling factor (0.0-1.0),
        interpolate between LED "off" color (at 0.0) and this item's fully-
        lit color (at 1.0) and set pixel to the result."""
        self.ring[pixel] = (
            (int(self.color[0] * scale) << 16)
            | (int(self.color[1] * scale) << 8)
            | int(self.color[2] * scale)
        )

    def iterate(self, xyz):
        """Given an accelerometer reading, run one cycle of the pendulum
        physics simulation and render the corresponding LED ring."""
        # Minus here is because LED pixel indices run clockwise vs. trigwise.
        # 0.05 is just an empirically-derived scaling fudge factor that looks
        # good; smaller values for more sluggish rings, higher = more twitch.
        self.momentum = (
            self.momentum * self.friction
            - (math.cos(self.angle) * xyz[2] + math.sin(self.angle) * xyz[0]) * 0.05
        )
        self.angle += self.momentum

        # Scale pendulum angle into pixel space
        midpoint = self.angle * 12 / math.pi % 24
        # Go around the whole ring, setting each pixel based on proximity
        # (this is also to erase the prior position)...
        for i in range(24):
            dist = abs(midpoint - i)  # Pixel to pendulum distance...
            if dist > 12:  #            If it crosses the "seam" at top,
                dist = 24 - dist  #      take the shorter path.
            if dist > 5:  #             Not close to pendulum,
                self.ring[i] = 0  #      erase pixel.
            elif dist < 2:  #           Close to pendulum,
                self.interp(i, 1.0)  #   solid color
            else:  #                    Anything in-between,
                self.interp(i, (5 - dist) / 3)  # interpolate


# List of pendulum objects, of which there are two: one per glasses ring
pendulums = [
    Pendulum(glasses.left_ring, (0, 20, 50)),  # Cerulean blue,
    Pendulum(glasses.right_ring, (0, 20, 50)),  # 50 is plenty bright!
]


def make_color(r, g, b, scale=1):
    return (int(r * scale) << 16 | int(g * scale) << 8 | int(b * scale))

def move_pixel(pixel, direction):
    pixel = pixel + direction
    if pixel < 0:
        pixel = 23
    if pixel > 23:
        pixel = 0
    return pixel

def outside_loop(state, glasses):
    # Remove the old one
    pixel = state["pixel"]
    ring = state["ring"]
    direction = state["direction"]
    ring[pixel] = make_color(0, 0, 0)

    # Adjust for this state
    if ring == glasses.left_ring and pixel == 5:
        ring = glasses.right_ring
        pixel = 18 
        direction = -1
    elif ring == glasses.right_ring and pixel == 19:
        ring = glasses.left_ring
        pixel = 6
        direction = 1
    else:
        pixel = move_pixel(pixel, direction)

    ring[pixel] = make_color(0, 255, 0)

    # update state
    state["ring"] = ring
    state["pixel"] = pixel
    state["direction"] = direction

# MAIN LOOP ---------

while True:

    # The try/except here is because VERY INFREQUENTLY the I2C bus will
    # encounter an error when accessing either the accelerometer or the
    # LED driver, whether from bumping around the wires or sometimes an
    # I2C device just gets wedged. To more robustly handle the latter,
    # the code will restart if that happens.
    try:

        accel = lis3dh.acceleration
        things = [(50, outside_loop)]
        last_run = [0]
        thing_state = [{
            "pixel": 0,
            "scale": 0.1,
            "ring": glasses.left_ring,
            "direction": 1
        }]

        last_run[0] = supervisor.ticks_ms()
        while True:
            for (freq, operation) in things:
                delta = supervisor.ticks_ms() - last_run[0];
                if delta > freq:
                    last_run[0] = supervisor.ticks_ms()
                    operation(thing_state[0], glasses)
                    glasses.show();

            # time.sleep(0.05);


    # See "try" notes above regarding rare I2C errors.
    except OSError:
        supervisor.reload()
