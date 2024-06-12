"""
shared_data.py

Defines the SharedData singleton class used for shared variables in
multithreading.
"""
from enum import Enum
import threading
import pigpio
import time
from subsystems.graph import Graph


class DECISION_STATUS(Enum):
    """
    Represents the decision status of the robot.

    Attributes:
        GOAL (int): Represents the robot goal-seeking status.
        EXPLORE (int): Represents the robot is exploring.
        MANUAL (int): Represents the robot is in manual mode.
    """
    GOAL = 0
    EXPLORE = 1
    MANUAL = 2



class SharedData:
    """
    Holds the shared variables between threads, including the lock.

    Attributes:
        lock (threading.Lock): Locks the shared data.
        graph (graph.Graph): Represents the graph.
        io (pigpio.pi): Represents the pigpio instance.
        decision_status (DECISION_STATUS): Represents the decision status.
        goal (int, int): Represents the goal coordinates.
        pause (bool): Indicates whether the execution is paused.
        step (bool): Indicates whether the execution is in step mode.
        direction: Represents the turn direction.
        save (bool): Indicates whether to save the graph.
        show (bool): Indicates whether to show the graph.
        clear (bool): Indicates whether to clear the graph.
        quit (bool): Indicates whether to quit the program.
        pose (bool): Indicates whether to update the pose.
        pose_buf (int, int, int): Pose buffer.
        distances (float, float, float): Ultrasound sensor readings.
    """

    def __init__(self):
        self.lock: threading.Lock = threading.Lock()
        self.graph: Graph = Graph()
        self.io: pigpio.pi = pigpio.pi()

        self.decision_status: DECISION_STATUS = DECISION_STATUS.EXPLORE
        self.goal: tuple[int, int] = None
        self.pause: bool = True
        self.step: bool = True
        self.lost: bool = False

        self.direction: int = None

        self.save: bool = False
        self.show: bool = False
        self.clear: bool = False
        self.quit: bool = False

        self.pose: bool = False
        self.pose_buf: tuple[int, int, int] = (0, 0, 0)

        self.distances: tuple[float, float, float] = (float('inf'), float('inf'), float('inf'))

        self.robotx = 0
        self.roboty = 0
        self.robotheading = 0


    def pause_step_block(self, check_period: float=0.1):
        """
        Blocks until the pause is False or the step is True. Checks every
        check_period seconds.

        Args:
            check_period (float): The period to check the pause and step.

        Returns:
            None
        """
        def get_pause_step():
            with self:
                pause = self.pause
                step = self.step
            return pause, step
        pause, step = get_pause_step()
        while pause and (not step):
            pause, step = get_pause_step()
            time.sleep(check_period)
        with self:
            self.step = False


    def __enter__(self):
        self.lock.acquire()

    def __exit__(self, exc_type, exc_value, traceback):
        self.lock.release()

    def __str__(self):
        return f"SharedData(decision_status={self.decision_status}, goal={self.goal}, pause={self.pause}, step={self.step}, save={self.save}, show={self.show}, clear={self.clear}, quit={self.quit}, pose={self.pose}, pose_buf={self.pose_buf}, direction={self.direction})"