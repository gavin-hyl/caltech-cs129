"""
linesensor.py

Defines the IR class and the LineSensor class, which contains logic and config 
for an individual IR sensor and a set of three IR sensors, respectively.
Team skeletons.
"""

import pigpio
from enum import Enum

class IR_T(Enum):
    FLOOR = 0
    LINE = 1

class IR:
    """
    An individual IR sensor.

    Attributes:
        pin: The pin the IR sensor is connected to.
        io: The GPIO interface object.
    """

    class PINS(Enum):
        LEFT = 18
        MID = 15
        RIGHT = 14

    def __init__(self, pin: int, io: pigpio.pi):
        """
        Initialize the IR sensor object.

        Args:
            pin: The pin the IR sensor is connected to.
            io: The GPIO interface object.
        """
        io.set_mode(pin, pigpio.INPUT)
        self.io: pigpio.pi = io
        self.pin: int = pin

    def read(self) -> int:
        """
        Reads the IR sensor"s results.

        Args:
            None

        Returns:
            1 if the sensor detects a line, 0 otherwise.
        """
        return self.io.read(self.pin)


class LineSensor:
    """
    A set of three IR sensors, from left to right.

    Attributes:
        ir_left: The left IR sensor.
        ir_mid: The middle IR sensor.
        ir_right: The right IR sensor.
    """

    def __init__(self, io: pigpio.pi) -> None:
        """
        Initialize the LineSensor object, which contains three IR sensors.

        Args:
            io: The GPIO interface object.
        """
        self.ir_left: IR = IR(IR.PINS.LEFT.value, io)
        self.ir_mid: IR = IR(IR.PINS.MID.value, io)
        self.ir_right: IR = IR(IR.PINS.RIGHT.value, io)

    def read(self) -> tuple[int, int, int]:
        """
        Reads the three IR sensors on the robot.

        Args:
            None

        Returns:
            A tuple of the IR sensor"s results, from left to right.
        """
        return (self.ir_left.read(), self.ir_mid.read(), self.ir_right.read())


def ir_test():
    """ Test function for the IR sensor. """
    import time
    io = pigpio.pi()
    sensor = LineSensor(io)
    last_read = sensor.read()
    start_time = time.time()
    try:
        while True:
            this_read = sensor.read()
            if last_read != this_read:
                last_read = this_read
                print(f"time: {time.time() - start_time}, reading: {this_read}")
    except KeyboardInterrupt:
        print("Stopping...")
    io.stop()


if __name__ == "__main__":
    ir_test()