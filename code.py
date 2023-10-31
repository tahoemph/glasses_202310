# pyright: reportShadowedImports=false
# SPDX-FileCopyrightText: 2021 Phil Burgess for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
This version started out as GOOGLY EYES for Adafruit EyeLight LED glasses + driver. Pendulum physics
That was Adapted from Bill Earl's STEAM-Punk Goggles project: https://learn.adafruit.com/steam-punk-goggles
"""
import math
import random
from random import randint
import time

import board
import busio
import supervisor

import adafruit_lis3dh
import adafruit_is31fl3741
from adafruit_is31fl3741.adafruit_ledglasses import LED_Glasses
from adafruit_is31fl3741.adafruit_rgbmatrixqt import Adafruit_RGBMatrixQT

# HARDWARE SETUP ----

# Shared by both the accelerometer and LED controller
i2c = busio.I2C(board.SCL, board.SDA, frequency=1_000_000)  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Initialize the accelerometer
# lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c)

def make_color(r, g, b, scale=1):
    return (int(r * scale) << 16 | int(g * scale) << 8 | int(b * scale))

def unmake_color(color):
    return ((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF)

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

def masomenos():
    while True:
        val = randint(-1, 1)
        if val != 0:
            return val

class GlassesRing:
    def __init__(self, i2c):
        self.glasses = LED_Glasses(i2c, allocate=adafruit_is31fl3741.MUST_BUFFER)
        self.glasses.show() # clear it up

    def left_ring(self):
        return self.glasses.left_ring

    def right_ring(self):
        return self.glasses.right_ring

    def pixel(self, x, y, color):
        self.glasses.pixel(x, y, color)

    def show(self):
        self.glasses.show()

class Trail:
    def __init__(self, glasses, length):
        self.path = []
        self.pixel = 5
        self.glasses = glasses
        self.ring = self.glasses.left_ring()
        self.direction = 1
        self.length = length
        self.last_run = supervisor.ticks_ms()

    def tick(self):
        now = supervisor.ticks_ms()
        if now - self.last_run >= 100:
            self.step()
            self.last_run = now

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
        if self.ring == self.glasses.left_ring() and self.pixel == 5:
            self.ring = self.glasses.right_ring()
            self.pixel = 18 
            self.direction = -1
        elif self.ring == self.glasses.right_ring() and self.pixel == 19:
            self.ring = self.glasses.left_ring()
            self.pixel = 6
            self.direction = 1
        else:
            self.pixel = move_pixel(self.pixel, self.direction)

        if self.ring == self.glasses.left_ring():
            color = store_color(255, 0, 0)
        else:
            color = store_color(0, 255, 0)
        self.ring[self.pixel] = make_color(*color)
        self.path.insert(0, (self.ring, self.pixel, color))

class Pixel:
    def __init__(self, x, y, color, direction = 1):
        self.x = x
        self.y = y
        self.color = color
        self.direction = direction

class Runner:
    def __init__(self, glasses):
        self.glasses = glasses
        self.pixels = None
        self.last_pixels = None
        self.last_run = supervisor.ticks_ms()
        self._reset = False

    def tick(self):
        now = supervisor.ticks_ms()
        if now - self.last_run >= 1000:
            if self.pixels == None or len(self.pixels) == 0:
                self.reset()
            self.step()
            self.last_run = now

    def reset(self):
        if self._reset:
            return
        self._reset = True
        self.pixels = []
        self.pixels.append(Pixel(8, 3, make_color(0, 255, 0), -1))
        self.pixels.append(Pixel(9, 3, make_color(0, 255, 0), 1))

    def no_match(self, p, pixels):
        for pixel in pixels:
            if p == pixel:
                continue
            if p.x == pixel.x and p.y == pixel.y and p.direction == pixel.direction:
                return False
        return True

    def step(self):
        if self.pixels == None or len(self.pixels) == 0:
            return
        if self.last_pixels is not None:
            for pixel in self.last_pixels:
                self.glasses.pixel(pixel.x, pixel.y, make_color(0, 0, 0))
        self.last_pixels = [Pixel(x.x, x.y, x.color, x.direction) for x in self.pixels]
        for pixel in self.pixels:
            self.glasses.pixel(pixel.x, pixel.y, pixel.color)
            pixel.x += pixel.direction
            pixel.y = pixel.y + randint(-1, 1)
            if pixel.y < 0:
                pixel.y = 0
            if pixel.y > 4:
                pixel.y = 4
        new_pixels = []
        for pixel in self.pixels:
            new_pixels.append(Pixel(pixel.x, pixel.y + masomenos(), self.next_color(pixel.color), pixel.direction))
        self.pixels = self.pixels + new_pixels
        self.pixels = [p for p in self.pixels if (p.x > 0 and p.x < 17) and (p.y >= 0 and p.y <= 4)]
        self.pixels = [p for p in self.pixels if self.no_match(p, self.pixels)]

    def next_color(self, color):
        components = unmake_color(color)
        new_color = [components[0] + randint(0, 1) * 25, components[1] + randint(-1, 0) * 25, components[2] + masomenos() * 2]
        new_color = [min(max(comp, 0), 255) for comp in new_color]
        print(f'2{components} {new_color}')
        return make_color(*new_color)

    def clear(self):
        pass

class Explosion:
    def __init__(self, glasses, x, y, color, direction):
        self.glasses = glasses
        self.pixels = None
        self.last_pixels = None
        self.last_run = supervisor.ticks_ms()
        self.start_x = x
        self.start_y = y
        self.start_color = color
        self.start_direction = direction

    def tick(self):
        now = supervisor.ticks_ms()
        if now - self.last_run >= 100:
            if self.pixels == None or len(self.pixels) == 0:
                print("reset")
                self.reset()
            self.step()
            self.last_run = now

    def reset(self):
        x = self.start_x
        y = self.start_y
        color = self.start_color
        direction = self.start_direction
        self.pixels = [
                Pixel(x, y, color, direction),
                Pixel(x - 1, y, color, direction),
                Pixel(x + 1, y, color, direction),
                Pixel(x, y + 1, color, direction),
                Pixel(x, y - 1, color, direction)
        ]

    def no_match(self, p, pixels):
        for pixel in pixels:
            if p == pixel:
                continue
            if p.x == pixel.x and p.y == pixel.y and p.direction == pixel.direction:
                return False
        return True

    def step(self):
        if self.pixels == None or len(self.pixels) == 0:
            return
        if self.last_pixels is not None:
            for pixel in self.last_pixels:
                self.glasses.pixel(pixel.x, pixel.y, make_color(0, 0, 0))
        self.last_pixels = [Pixel(x.x, x.y, x.color, x.direction) for x in self.pixels]
        for pixel in self.pixels:
            self.glasses.pixel(pixel.x, pixel.y, pixel.color)
            pixel.x += pixel.direction
        self.pixels = [p for p in self.pixels if (p.x > 0 and p.x < 17) and (p.y >= 0 and p.y <= 4)]

    def clear(self):
        pass

while True:

    # The try/except here is because VERY INFREQUENTLY the I2C bus will
    # encounter an error when accessing either the accelerometer or the
    # LED driver, whether from bumping around the wires or sometimes an
    # I2C device just gets wedged. To more robustly handle the latter,
    # the code will restart if that happens.
    glasses = GlassesRing(i2c)
    trail = Trail(glasses, 4)
    explosion_l = Explosion(glasses, 5, 3, make_color(0, 255, 0), -1)
    explosion_r = Explosion(glasses, 12, 3, make_color(255, 0, 0), 1)
    last_run = supervisor.ticks_ms()
    try:
        # frequency to call setup, frequency to call step, clear to call inbetween
        # the operation returns True when it is done
        things = (trail.tick, explosion_r.tick, explosion_l.tick)
        while True:
            now = supervisor.ticks_ms()
            if now - last_run >= 10:
                for op in things:
                    op()
                glasses.show()
                last_run = now

    # See "try" notes above regarding rare I2C errors.
    except OSError:
        supervisor.reload()
