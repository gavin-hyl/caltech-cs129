"""
odometry.py

Defines the Odometry class, which contains logic and config for the sensor
odometry for the robot.

Team skeletons.
"""

import numpy as np

class Odometry:
    """
    The Odometry class contains logic and helper functions for reading and
    tracking the robot's position and heading.

    Parameters:
        x: The x-coordinate of the robot (defined as grid units in the direction
           of the robot's starting heading).
        y: The y-coordinate of the robot (defined as grid units in the direction
           left to the robot's starting heading).
        h_pos: The heading of the robot (int between 0-7).
    """

    def __init__(self, x:int=0, y:int=0, h:int=0) -> None:
        """
        Initializes the Odometry object.

        Args:
            x: The x-coordinate of the robot.
            y: The y-coordinate of the robot.
            h: The heading of the robot.
        """
        self.x = x
        self.y = y
        self.h = h
    

    def pos_xy(self) -> tuple[int, int]:
        return self.x, self.y
    
    
    def pos_xyh(self) -> tuple[int, int, int]:
        return self.x, self.y, self.h


    def calc_move(self, heading: int = None) -> tuple[int, int]:
        """
        Calculates the movement of the robot based on the heading.

        Args:
            heading: The heading of the robot, measured ccw from the
                     positive x-axis and ranging from 0 to 7 in even increments.

        Returns:
            The new x and y position of the robot.
        """
        if heading is None:
            heading = self.h
        dxy = Odometry.h_dxy(heading)
        self.x += dxy[0]
        self.y += dxy[1]
        return self.x, self.y


    def calc_turn(self, turn: int) -> int:
        """
        Calculates the new heading of the robot based on the turn.

        Args:
            turn: The turn of the robot, measured in number of 45-degree increments.

        Returns:
            The new heading of the robot.
        """
        self.h = (self.h + turn) % 8    # always ensure positive
        return self.h


    def calc_u_turn(self) -> int:
        """
        Calculates the new heading of the robot after a U-turn.

        Returns:
            The new heading of the robot.
        """
        self.h = Odometry.h_opposite(self.h)
        return self.h


    @staticmethod
    def h_dxy(h_pos: int) -> tuple[int, int]:
        """
        Converts the current heading to an x and y direction.

        Arguments:
            heading: the heading of the robot, measured in a number of 45-degree
                     increments counterclockwise.
        
        Returns:
            A tuple containing the x and y direction.
        """
        angle = h_pos * np.pi / 4
        return np.sign(round(-np.sin(angle))), np.sign(round(np.cos(angle)))


    @staticmethod
    def h_opposite(h_pos: int) -> int:
        """
        Computes the opposite heading position.

        Args:
            h_pos: The current heading position.

        Returns:
            The opposite heading position.
        """
        return (h_pos + 4) % 8


    @staticmethod
    def h_diff(h1: int, h2: int) -> int:
        """
        Computes the positive difference between two heading positions.

        Args:
            h1: The first heading position.
            h2: The second heading position.
        
        Returns:
            The positive difference between the two heading positions.
        """
        return (h1 - h2) % 8
    
    
    @staticmethod
    def h_abs_cap(h: int) -> int:
        """
        Converts the heading to a value between -4 and 3

        Args:
            h: The heading of the robot.

        Returns:
            The capped heading.
        """
        if abs(h) > 4 or h == 4:
            return -np.sign(h) * (8 - abs(h))
        return h
    

    @staticmethod
    def h_to(x1: int, y1: int, x2: int, y2: int) -> int:
        """
        Computes the heading from one point to another.

        Args:
            x1: The x-coordinate of the starting point.
            y1: The y-coordinate of the starting point.
            x2: The x-coordinate of the ending point.
            y2: The y-coordinate of the ending point.
        
        Returns:
            The heading from the starting point to the ending point.
        """
        angle = np.arctan2(y2 - y1, x2 - x1)
        return (Odometry.angle_to_heading(angle)[0]-2) % 8


    @staticmethod
    def angle_to_heading(angle: float, floor: int = 0) -> tuple[int, float]:
        """
        Converts an angle to a heading.

        Args:
            angle: The angle in radians.
            floor: The minimum value of the heading.

        Returns:
            A tuple containing the heading and fractional error.
        """
        h_raw = angle / (2 * np.pi) * 8
        h_round = round(h_raw)
        if abs(h_round) < floor:
            h_round = floor * np.sign(h_round)
        return h_round, abs(h_raw - h_round)