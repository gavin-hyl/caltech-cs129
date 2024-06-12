"""
drive.py

Defines the DriveSystem class, which contains logic and config for the low-level
drive logic for the robot.
Team skeletons.
"""

import pigpio
from enum import Enum

class Motor:
    """
    Abstract representation of a motor. Contains the pin numbers and the PWM
    configurations.

    Attributes:
        io: the GPIO interface of the Pi
        pin_p: see pin_p.
        pin_n: pin_n HIGH/pinL LOW moves forward, while pin_n LOW/pinL HIGH moves backwards.
        pwm_max: the maximum integer for the PWM duty cycle
    """
    class PINS(Enum):
        LEFT_P = 8
        LEFT_N = 7
        RIGHT_P = 6
        RIGHT_N = 5

    def __init__(self,
                 pin_p: int,
                 pin_n: int,
                 io: pigpio.pi,
                 freq: int = 1000,
                 pwm_max: int = 255):
        """
        Initialize the motor with the two pins and the frequency. Sets the two
        pins to OUTPUT, and initializes the PWM and sets its frequency.

        Args:
            pin_p: the positive pin of the motor. When this pin is activated, the
                  robot moves forward.
            pin_n: the negative pin of the motor
            freq: the PWM frequency
            pwm_max: the maximum integer representing of the duty cycle
        """
        self.io: pigpio.pi = io
        self.pin_p: int = pin_p
        self.pin_n: int = pin_n
        self.pwm_max: int = pwm_max
        for pin in [pin_p, pin_n]:
            self.io.set_mode(pin, pigpio.OUTPUT)
            self.io.set_PWM_frequency(pin, freq)
            self.io.set_PWM_range(pin, pwm_max)
        self.stop()


    def stop(self) -> None:
        """ Stops the motor. """
        self.set_speed(0)


    def set_speed(self, level: float) -> None:
        """
        Drives the motor using PWM.

        Args:
            level: a float from -1.0 to 1.0, max negative speed to max positive.
                   Values out of this range will be clipped.

        Returns:
            None
        """

        clipped_level = max(-1, min(1, level))
        pwm_level = int(clipped_level * self.pwm_max)
        if pwm_level >= 0:
            self.io.set_PWM_dutycycle(self.pin_p, pwm_level)
            self.io.set_PWM_dutycycle(self.pin_n, 0)
        else:
            self.io.set_PWM_dutycycle(self.pin_n, abs(pwm_level))
            self.io.set_PWM_dutycycle(self.pin_p, 0)


class DriveSystem:
    """
    Representation of the drive system of the robot. Contains the logic for the
    initialization of the motors and low-level drive logic.

    Attributes:
        motor_left: the left motor of the robot
        motor_right: the right motor of the robot
        DIRECTION_OMEGA_TO_MOTOR_SPEEDS: a dictionary mapping the direction and
                                         omega to motor speeds
    """
    MAX_OMEGA = 5


    def __init__(self, io: pigpio.pi):
        """
        Instantiate the motors.

        Args:
            io: The GPIO interface object.
        """
        self.motor_left: Motor = Motor(Motor.PINS.LEFT_P.value, Motor.PINS.LEFT_N.value, io)
        self.motor_right: Motor = Motor(Motor.PINS.RIGHT_P.value, Motor.PINS.RIGHT_N.value, io)
        self.motor_compensation: float = 1.07 #! change this

        # Direction (-1, 0, 1), omega (0 to 5) mapping to speed (left, right, -1 to 1)
        # m is the multiplier
        self._DIRECTION_OMEGA_TO_MOTOR_SPEEDS: dict[tuple[int,int], tuple[int, int]] = {
            (0,0): (0.72 , 0.8),   # Straight
            (0,-1): (-0.77 , -0.75),   # Straight

            (-1,1): (0.9 , 0.8),     # Right Veer
            (-1,2): (0.8 , 0.7),    # Right Steer
            (-1,3): (0.85 , 0.6),    # Right Turn
            (-1,4): (0.8 , 0),      # Right Hook
            (-1,5): (0.75, -0.77),  # Right Spin

            (1,1): (0.7 , 0.8),      # Left Veer
            (1,2): (0.54 , 0.75),      # Left Steer
            (1,3): (0.45 , 0.85),     # Left Turn
            (1,4): (0 , 0.85),       # Left Hook
            (1,5): (-0.75 , 0.77)    # Left Spin
        }


    def drive(self, direction: int, omega: int, multiplier: int = 1) -> None:
        """
        Drive the robot in a given direction with a given turn speed.

        Args:
            direction: The direction to drive the robot in, must be either -1, 0,
                       or 1. 1 is turning to CCW, 0 is straight, -1 is CW.
            omega:     The strength of the turn, must be between 0 and 5.
                       The larger omega the faster the robot will turn.

        Returns:
            None
        """
        motor_speeds = self._DIRECTION_OMEGA_TO_MOTOR_SPEEDS[(direction, omega)]
        self.motor_left.set_speed(motor_speeds[0] * multiplier * self.motor_compensation)
        self.motor_right.set_speed(motor_speeds[1] * multiplier * self.motor_compensation)

    def turn_max(self, direction: int) -> None:
        """
        Turn the robot in a given direction at the maximum speed.

        Args:
            direction: The direction to turn the robot in, must be either -1 or 1.
                       1 is turning to CCW, -1 is CW.

        Returns:
            None
        """
        self.drive(direction, self.MAX_OMEGA)

    def set_speed(self, speed_l: int, speed_r: int) -> None:
        """
        Exposes the motor PWM control.

        Args:
            pwm_l: the speed value for the left motor (-1 to 1)
            pwm_r: the speed value for the right motor (-1 to 1)

        Returns:
            None
        """
        self.motor_left.set_speed(speed_l)
        self.motor_right.set_speed(speed_r)


    def stop(self) -> None:
        """ Stop both motors. """
        self.motor_left.stop()
        self.motor_right.stop()


def test() -> None:
    """ Test function for the drive system. """
    import time
    io = pigpio.pi()
    drivetrain = DriveSystem(io)
    try:
        drivetrain.drive(0,0)
        time.sleep(4)
        drivetrain.stop()
    except:
        drivetrain.stop()
    io.stop()


if __name__ == "__main__":
    test()