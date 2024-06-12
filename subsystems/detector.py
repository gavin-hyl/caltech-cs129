'''
detector.py

Defines the Detector class, which contains logic and config for filtering sensor
readings using a first-order low-pass filter, and parsing it into states.
Team skeletons.
'''

import time
import numpy as np

class Detector:
    def __init__(self, alpha: float, y_init: float = 0.5, thresh: tuple[float, float] = (0.3, 0.7)) -> None:
        '''
        Initializes the Detector object.

        Args:
            alpha: The time constant of the filter.
            y_init: The initial value of the filter.
            thresh: A tuple of thresholds (low, high).
        '''
        self.alpha: float = alpha
        self.y: float = y_init
        self.y_low: float = thresh[0]
        self.y_high: float = thresh[1]
        self.state = 0 if y_init < self.y_low else 1
        self.t = None
    
    
    def update(self, raw: float) -> float:
        '''
        Updates the filter with a new value, with the equation
        y_t = y_{t-1} + dt/alpha * (raw - y_{t-1}).

        Args:
            raw: The new value to update the filter with.

        Returns:
            The filtered value.
        '''
        # Update time
        if self.t is None:
            self.y = raw
            self.t = time.time()
        else:
            new_t = time.time()
            self.y += (new_t - self.t) / self.alpha * (raw - self.y)
            self.t = new_t
        if abs(self.y) < self.y_low:
            self.state = 0
        elif abs(self.y) > self.y_high:
            self.state = np.sign(self.y)
        return self.state