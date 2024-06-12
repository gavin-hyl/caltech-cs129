"""
movement.py

Defines the Movement class, which contains logic and config for the mid-level
movement control of the robot.
Team skeletons.
"""

if __name__ == "__main__":
    from hardware_interface.drive import DriveSystem
    from hardware_interface.linesensor import LineSensor
    from hardware_interface.anglesensor import AngleSensor
    from detector import Detector
    from odometry import Odometry
else:
    from subsystems.detector import Detector
    from subsystems.hardware_interface.drive import DriveSystem
    from subsystems.hardware_interface.linesensor import LineSensor
    from subsystems.hardware_interface.anglesensor import AngleSensor
    from subsystems.odometry import Odometry
    from subsystems.timer import Timer
    from shared import SharedData


from pigpio import pi
import time
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum

class LINE_FOLLOW_T(Enum):
    UNKNOWN = 0
    EOL = 1
    INTERSECT = 2
    BLOCK = 3
    T_INTERSECT = 4



class Movement:
    """
    The Movement class contains logic for mid-level control of robot movement.
    Integrates the DriveSystem, LineSensor, and AngleSensor objects, and
    provides methods for line following, turning, and pulling forward.

    Attributes:
        drivetrain: The DriveSystem object, responsible for low-level movement control.
        line_sensor: The LineSensor object, responsible for reading line sensor data.
        angle_sensor: The AngleSensor object, responsible for reading the robot's angle.
    """
    def __init__(self, io: pi, calibrate: bool=True, shared: SharedData = None) -> None:
        self.drivetrain = DriveSystem(io)
        self.line_sensor = LineSensor(io)
        self.angle_sensor = AngleSensor(io)
        self.shared = shared
        try:
            if calibrate:
                self.calibrate_mag()
        except KeyboardInterrupt:
            print("[movement] keyboard interrupt")
        except:
            print("[movement] unknown error")
        finally:
            self.drivetrain.stop()


    def calibrate_mag(self):
        """
        Calibrates the magnetometers by turning and gathering the correct range
        of possible values. Saves the correct range of values to the angle
        sensor instance.
        """
        CALIBRATION_TIME_S = 3
        t_cal_start = time.time()
        self.drivetrain.turn_max(1)
        bx_max, by_max = 0, 0
        bx_min, by_min = 255, 255
        while t_cal_start + CALIBRATION_TIME_S > time.time():
            b_readings = self.angle_sensor.adc.read_all()
            x_filtered = b_readings[0]
            y_filtered = b_readings[1]
            bx_max = max(bx_max, x_filtered)
            by_max = max(by_max, y_filtered)
            bx_min = min(bx_min, x_filtered)
            by_min = min(by_min, y_filtered)
        self.angle_sensor.bx_max, self.angle_sensor.bx_min = bx_max, bx_min
        self.angle_sensor.by_max, self.angle_sensor.by_min = by_max, by_min
        self.turn_to_next(1)


    def plot_mag(self):
        """ Plots the magnetometer values. """
        magx = []
        magy = []
        # calibrate the magnetometers
        CALIBRATE_TIME = 5
        t_cal_start = time.time()
        self.drivetrain.turn_max(1)
        while t_cal_start + CALIBRATE_TIME > time.time():
            magx.append(self.angle_sensor.read_norm_adc()[0])
            magy.append(self.angle_sensor.read_norm_adc()[1])
        plt.plot(magx, label="x")
        plt.plot(magy, label="y")
        plt.legend()
        self.drivetrain.stop()
        plt.savefig(r"/home/robot/project/magnetometer_plot.png")


    def pull_forward(self) -> int:
        """
        Pulls the robot forward to center it at the intersection.

        Returns:
            The state of the line sensor, 1 if a line exists, 0 otherwise.
        """
        PULL_FORWARD_T = 0.35   #! make this value a little larger than needed - emergency u-turns
        PULL_FORWARD_T_STOP = 0.1
        line_exists_detector = Detector(alpha=0.1, y_init=0)
        t0 = time.time()
        while time.time() < t0 + PULL_FORWARD_T + PULL_FORWARD_T_STOP:
            mid = self.line_sensor.read()[1]
            if time.time() < t0 + PULL_FORWARD_T:
                if mid:
                    self.line_lock(0)
                else:
                    self.drivetrain.drive(0, 0)
            else:
                line_exists_detector.update(mid)
                self.drivetrain.stop()
        self.drivetrain.stop()
        time.sleep(0.15)
        return line_exists_detector.state


    def in_tunnel(self):
        us_readings = self.shared.distances
        TUNNEL_MIN = 0.05
        TUNNEL_MAX = 0.45
        # print(us_readings)
        return TUNNEL_MIN < us_readings[0] < TUNNEL_MAX and \
               TUNNEL_MIN < us_readings[2] < TUNNEL_MAX

    def line_lock(self, prev_stray = 0):
        STRAY_TO_OMEGA = [0, 2, 3, 4, 5]
        ir_readings = self.line_sensor.read()
        if ir_readings != (0, 0, 0):
            stray = (ir_readings[2] - ir_readings[0]) / sum(ir_readings)
            if ir_readings == (1, 0, 1):
                stray = prev_stray
            self.drivetrain.drive(direction=-np.sign(stray),
                                    omega=STRAY_TO_OMEGA[int(abs(stray) * 2)])
            return stray
        else:
            return 0


    def line_follow(self, obstacle_wait_t: float = 3, blip_avoid_t: float=1.8) -> LINE_FOLLOW_T:
        """
        Follows a line using the LineSensor and DriveSystem objects.

        Args:
            None

        Returns:
            EOL if the end of line, INTERSECT if an intersection is detected,
            BLOCK if an obstacle is detected, and T_INTERSECT if a tunnel
            intersection is detected.
        """
        # Maps the absolute value of the stray variable to turn strength.
        OBSTACLE_D_MIN = 0.2
        TUNNEL_KP = 1.3
        TUNNEL_SLOWDOWN = 0.9
        NOMINAL_SPEEDS = self.drivetrain._DIRECTION_OMEGA_TO_MOTOR_SPEEDS.get((0, 0))
        # Declare detectors
        line_stray_detector = Detector(alpha=0.05, y_init=0)
        intersection_detector = Detector(alpha=0.07, y_init=0)
        eol_detector = Detector(alpha=0.2, y_init=0)
        obstacle_detector = Detector(alpha=0.01, y_init=0)
        tunnel_detector = Detector(alpha=0.5, y_init=0)
        obstacle_wait_start = 0
        timer = Timer()
        timer.start()
        stray = 0
        while True:
            # handle obstacles
            if obstacle_detector.update(1 if self.shared.distances[1] < OBSTACLE_D_MIN else 0) == 1:
                if obstacle_wait_start == 0:
                    obstacle_wait_start = time.time()
                elif obstacle_wait_start + obstacle_wait_t < time.time() and not self.in_tunnel():
                    return LINE_FOLLOW_T.BLOCK
                self.stop()
            else:
                obstacle_wait_start = 0
            if obstacle_wait_start:
                timer.pause()
                continue
            # obstacles clear, do line-follow
            timer.start()
            ir_readings = self.line_sensor.read()
            us_readings = self.shared.distances
            if ir_readings != (0, 0, 0):
                eol_detector.update(0)
                stray = self.line_lock(stray)
                line_stray_detector.update(stray)
                tunnel_detector.update(0)
                if intersection_detector.update((1 if ir_readings==(1, 1, 1) else 0)) == 1 and (timer.check() > blip_avoid_t):
                    self.stop()
                    return LINE_FOLLOW_T.T_INTERSECT if tunnel_detector.state else LINE_FOLLOW_T.INTERSECT
            elif ir_readings == (0, 0, 0) and not self.in_tunnel():
                stray = line_stray_detector.state
                if eol_detector.update(1) == 1:
                    self.stop()
                    return LINE_FOLLOW_T.EOL
                elif abs(stray) != 0:
                    self.drivetrain.turn_max(-np.sign(stray))
                else:
                    self.drivetrain.drive(0, 0)
                intersection_detector.update(0)
                tunnel_detector.update(0)
            else:
                eol_detector.update(0)
                error = us_readings[0] - us_readings[2]
                pwm_left = NOMINAL_SPEEDS[0]  * TUNNEL_SLOWDOWN - TUNNEL_KP * error
                pwm_right = NOMINAL_SPEEDS[1] * TUNNEL_SLOWDOWN  + TUNNEL_KP * error
                self.drivetrain.set_speed(pwm_left, pwm_right)
                tunnel_detector.update(1)


    def turn_to_next(self, direction: int) -> int:
        """
        Turns the robot to the next line using the LineSensor and DriveSystem objects.

        Args:
            direction: The direction to turn in, must be either -1 or 1.

        Returns:
            The heading change as reported by the AngleSensor object.
        """
        turn_started = False
        turn_start_detector = Detector(alpha=0.02, y_init=1)
        turn_end_lead_detector = Detector(alpha=0.05, y_init=0, thresh=(0.6, 0.8))
        turn_end_mid_detector = Detector(alpha=0.05, y_init=0, thresh=(0.6, 0.8))
        # turning phase
        angle_i = self.angle_sensor.read()
        turn_timer = Timer()
        turn_timer.start()
        self.drivetrain.drive(direction=direction, omega=5, multiplier=0.9)
        while True:
            ir_readings = self.line_sensor.read()
            mid = ir_readings[1]
            lead = ir_readings[1 - direction]
            lag = ir_readings[1 + direction]
            if turn_start_detector.update(mid) == 0:
                turn_started = True
            if (mid * lead or mid * lag) and turn_started:
                break
        turn_timer.pause()
        centering_detector = Detector(alpha=0.5, y_init=1)
        stray_detector = Detector(alpha=0.01, y_init=0)
        CENTERING_TIMEOUT = 2
        center_timer = Timer()
        center_timer.start()
        # centering phase
        while center_timer.check() < CENTERING_TIMEOUT: # locking in loop
            reading = self.line_sensor.read()
            if reading == (0, 0, 0):
                stray = stray_detector.state
                if stray != 0:
                    self.drivetrain.turn_max(-np.sign(stray))
            else:
                # abs(stray) is either 0, +/- 0.5, or +/- 1
                stray = (reading[2] - reading[0]) / sum(reading)
                stray_detector.update(stray)
                if stray != 0:
                    # self.drivetrain.turn_max(-np.sign(stray))
                    self.drivetrain.drive(direction=-np.sign(stray), omega=5, multiplier=0.8)
                elif centering_detector.update(abs(stray_detector.state)) == 0:
                    self.stop()
        self.stop()
        h_mag, h_mag_err = Odometry.angle_to_heading(self.angle_sensor.read() - angle_i, floor=1 if turn_timer.check() < 1.5 else 0)
        # return h_time if h_time_err < h_mag_err else h_mag
        return h_mag


    def turn_to_heading(self, h_now: int, h_goal: int) -> tuple[list[int], int]:
        """
        Turns the robot to a specified heading.

        Args:
            h_now: The current heading of the robot.
            h_goal: The desired heading of the robot.

        Returns:
            The list of streets encountered and the direction of the turn, and
            the direction of the turn.
        """
        h_diff = h_goal - h_now
        direction = np.sign(h_diff)
        if (capped_h_diff := Odometry.h_abs_cap(h_diff)) != h_diff:
            direction *= -1

        turned_h_diff = 0
        encountered_streets = []
        while True:
            turn_h = self.turn_to_next(direction)
            turned_h_diff = Odometry.h_abs_cap(turned_h_diff + turn_h)
            encountered_streets.append((turned_h_diff + h_now) % 8)
            if abs(turned_h_diff) >= abs(capped_h_diff) \
                or np.sign(turned_h_diff) != direction \
                or turned_h_diff == 0:
                return encountered_streets, direction


    def stop(self) -> None:
        """ Stops the drivetrain. """
        self.drivetrain.stop()


    def time_to_heading(self, delta_t: float, floor: int = 1) -> tuple[int, float]:
        """
        Calculates the absolute heading change given a time delta.

        Args:
            delta_t: The time delta.
            floor: The floor value for the heading change.

        Returns:
            The heading change, and the fractional error of the heading change.
        """
        AVERAGE_TIMES = [(0, 0.4), (0.65, 0.8), (0.95, 1.2), (1.45, 1.7)]
        for i, (t_low, t_high) in enumerate(AVERAGE_TIMES):
            if t_low < delta_t < t_high:
                if i == 0:
                    err = 0
                else:
                    max_diff = t_high - t_low
                    err = abs(delta_t - (t_low + t_high)/2) / max_diff
                return i+1, err
        return 0, float('inf')


def mag_cal_test():
    io = pi()
    move = Movement(io)
    try:
        move.calibrate_mag()
        print("finished calibration")
        move.plot_mag()
        while True:
            print(move.angle_sensor.read() / 2 / 3.1415 * 360)
    except:
        move.stop()
    io.stop()

if __name__ == "__main__":
    mag_cal_test()