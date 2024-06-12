"""
anglesensor.py

This file contaiins the AngleSensor and ADConverter classes. These classes are used
to interface with the magnetometer and process its data.
Team skeletons.
"""

import numpy as np
from pigpio import pi
import pigpio

class AngleSensor:
    """
    The AngleSensor is an abstraction of two magnetometers. The class contains
    logic for collecting magnetometer data and converting it to a robot heading.

    Attributes:
        adc: The ADConverter object.
        bx_max: The upper boundary of the x-direction magnetometer.
        by_max: The upper boundary of the y-direction magnetometer.
        bx_min: The lower boundary of the x-direction magnetometer.
        by_min: The upper boundary of the y-direction magnetometer.
    """

    def __init__(self, io: pi) -> None:
        """
        Initializes the AngleSensor object.

        Args:
            io: The GPIO interface object.
        """
        self.adc = ADConverter(io)
        self.bx_max = 164
        self.by_max = 251
        self.bx_min = 73
        self.by_min = 159


    def read(self) -> float:
        """
        Reads the angle of the robot. Left turns are expected to yield a positive
        angle difference.

        Returns:
            The angle of the robot in radians.
        """
        mag_x, mag_y = self.read_norm_adc()
        angle = np.arctan2(mag_y, mag_x)
        return -angle


    def read_norm_adc(self) -> tuple[int, int]:
        """
        Reads, normalizes, and returns the magnetometer values.
        """
        spreadx = self.bx_max - self.bx_min
        spready = self.by_max - self.by_min
        midx = spreadx/2 + self.bx_min
        midy = spready/2 + self.by_min
        readings = self.adc.read_all()
        mag_x = (readings[0] - midx) / spreadx * 2
        mag_y = (readings[1] - midy) / spready * 2
        return mag_x, mag_y



class ADConverter:
    """
    Abstract representation of an analog to digital converter.
    """
    MAG_LATCH = 27
    MAG_ADDR = 4
    MAG_RDY = 17
    MAG_MUX_X = 0
    MAG_MUX_Y = 1
    MAG_DAT = [9, 10, 11, 12, 22, 23, 24, 25]    # LSB first
    MAG_CALIBRATE_TIME = 4

    def __init__(self, io: pi, latch_p: int = MAG_LATCH, addr_p: int = MAG_ADDR,
                 ready_p: int = MAG_RDY, data_p: list[int] = MAG_DAT):
        """
        Initializes an ADConverter object using the provided pins.

        Arguments:
            io: the io instance of the pi
            latch_p: the latch pin number
            addr_p: the address pin number
            read_p: the "ready" pin number
            data_p: a list of 8 data pins in order of LSB to MSB
        """
        self.io = io
        self.latch_p = latch_p
        self.addr_p = addr_p
        self.ready_p = ready_p
        self.data_p = data_p

        self.io.set_mode(self.latch_p, pigpio.OUTPUT)
        self.io.set_mode(self.addr_p, pigpio.OUTPUT)
        self.io.set_mode(self.ready_p, pigpio.INPUT)
        for pin in data_p:
            self.io.set_mode(pin, pigpio.INPUT)


    def read(self, address: int) -> int:
        """
        Reads the ADC from a provided input address.

        Arguments:
            address: the address from which the reading is being fetched
                     (either 0 or 1).

        Returns:
            An integer that represents the reading.
        """
        self.io.write(self.latch_p, 0)  # set latch low
        self.io.write(self.addr_p, address)  # set address
        self.io.write(self.latch_p, 1)  # set latch high
        self.io.write(self.latch_p, 0)  # set latch low
        self.io.write(self.latch_p, 1)  # set latch high
        while self.io.read(self.ready_p) == 0:  # wait for ready signal
            pass
        reading = 0
        for bit_n, pin in enumerate(self.data_p):  # read pins 0-7
            reading += self.io.read(pin) << bit_n
        return reading


    def read_all(self) -> tuple[int, int]:
        """
        Returns readings from both addresses (0 then 1) as a tuple.
        """
        return self.read(0), self.read(1)


def test():
    """
    Prints the robot angle values in degrees.
    """
    io = pi()
    sensor = AngleSensor(io)
    while True:
        print(sensor.read() / 2 / 3.1415 * 360)


if __name__ == "__main__":
    test()