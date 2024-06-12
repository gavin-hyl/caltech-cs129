"""
proximitysensor.py

Defines the Ultrasound and ProximitySensor classes, which contain logic and
config for the low-level ultrasound sensor logic for the robot.

Team skeletons.
"""

import pigpio
import time

class Ultrasound:
    """
    An HC-SR04 ultrasound sensor.

    Attributes:
        io: GPIO interface object
        tx_pin: the trigger pin on the sensor
        rx_pin: the echo pin on the sensor

    """

    TIMEOUT_ms: float = 50
    TIMEOUT_us: float = TIMEOUT_ms * 10**3
    TIMEOUT_s: float = TIMEOUT_ms / 10**3

    class PINS():
        LEFT = (13, 16)
        MID = (19, 20)
        RIGHT = (26, 21)

    def __init__(self, io: pigpio.pi, tx_pin: int, rx_pin: int, first_trig: bool=True) -> None:
        """
        Constructor for the ultrasound sensor.

        Args:
            io: GPIO interface object
            tx_pin: the trigger pin on the sensor
            rx_pin: the echo pin on the sensor
            auto_trigger: whether or not to trigger and sleep when the constructor is
                          called to ensure there is a valid reading when get_dist
                          is called.
        """
        self.io = io
        self.rise_t = 0
        self.dist = float('inf')
        self.tx_pin = tx_pin
        self.rx_pin = rx_pin

        self.io.set_mode(tx_pin, pigpio.OUTPUT)
        self.io.set_mode(rx_pin, pigpio.INPUT)
        self._cbrise = self.io.callback(rx_pin, pigpio.RISING_EDGE, self._rising)
        self._cbfall = self.io.callback(rx_pin, pigpio.FALLING_EDGE, self._falling)

        if first_trig:
            self.trigger()
            time.sleep(self.TIMEOUT_s)


    def trigger(self) -> bool:
        """
        Triggers a pulse the ultrasound sensor. The sensor will refuse to
        trigger if the time since the last trigger is not longer than the
        mandatory wait time between triggers to avoid indistinguishable echos.

        Args:
            None

        Returns:
            True if the sensor was triggered, False otherwise.
        """
        wait_ticks = 50 * 10**3
        if (self.io.get_current_tick() < self.rise_t + wait_ticks):
            return False
        self.io.gpio_trigger(self.tx_pin, pulse_len=10)
        return True


    def _rising(self, pin, level, ticks) -> None:
        """
        Callback function for the rising edge on RX.
        Records the time.
        """
        self.rise_t = ticks


    def _falling(self, pin, level, ticks) -> None:
        """
        Callback function for the falling edge on RX.
        Converts the RX high time to a distance in meters.
        """
        if  (delta_t := ticks - self.rise_t) < 0:
            delta_t += 2**32
        if delta_t < Ultrasound.TIMEOUT_us:
            self.dist = delta_t * 340 / 2 / 10**6
        else:
            self.dist = float('inf')



class ProximitySensor:
    """
    A class to represent a proximity sensor using three ultrasound sensors.

    Attributes:
        us_left: The left ultrasound sensor
        us_mid: The middle ultrasound sensor
        us_right: The right ultrasound sensor
    """
    def __init__(self, io: pigpio.pi,
                 tx_left: int = Ultrasound.PINS.LEFT[0],
                 rx_left: int = Ultrasound.PINS.LEFT[1],
                 tx_mid: int = Ultrasound.PINS.MID[0],
                 rx_mid: int = Ultrasound.PINS.MID[1],
                 tx_right: int = Ultrasound.PINS.RIGHT[0],
                 rx_right: int = Ultrasound.PINS.RIGHT[1]) -> None:
        self.us_left = Ultrasound(io, tx_left, rx_left)
        self.us_mid = Ultrasound(io, tx_mid, rx_mid)
        self.us_right = Ultrasound(io, tx_right, rx_right)

    def trigger(self) -> tuple[bool, bool, bool]:
        TRIG_WAIT_s = self.us_left.TIMEOUT_s
        left_trig = self.us_left.trigger()
        time.sleep(TRIG_WAIT_s)
        mid_trig = self.us_mid.trigger()
        time.sleep(TRIG_WAIT_s)
        right_trig = self.us_right.trigger()
        time.sleep(TRIG_WAIT_s)
        return (left_trig, mid_trig, right_trig)

    def read(self) -> tuple[float, float, float]:
        return (self.us_left.dist, self.us_mid.dist, self.us_right.dist)

    def in_tunnel(self):
        us_readings = self.read()
        TUNNEL_MIN = 0.1
        TUNNEL_MAX = 0.3
        return TUNNEL_MIN < us_readings[0] < TUNNEL_MAX and \
               TUNNEL_MIN < us_readings[2] < TUNNEL_MAX


def ultrasound_test():
    io = pigpio.pi()
    sensor = ProximitySensor(io)
    while True:
        sensor.trigger()
        distances = sensor.read()
        time.sleep(0.1)
        print(distances)


def main(shared):
    proximity = ProximitySensor(shared.io)
    try:
        while not shared.quit:
            proximity.trigger()
            # FIXME: REMOVED LOCK ACQUIRE TEMPORARILY
            shared.distances = proximity.read()
    except KeyboardInterrupt:
        print("[trigger] keyboard interrupt")
    except Exception as e:
        print("[trigger] exception:", e)


if __name__ == '__main__':
    ultrasound_test()