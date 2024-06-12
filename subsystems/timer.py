import time

class Timer():
    """
    Simple timer class that can be started, paused, and checked. The timer
    is based on the time module, and should not be used for precise timing.

    Attributes:
        t (float): The time elapsed.
        start_t (float): The time when the timer was started.
        paused (bool): Indicates whether the timer is paused.
    """

    def __init__(self):
        """
        Initializes the timer.
        """
        self.t = 0
        self.start_t = 0
        self.paused = True

    def start(self) -> None:
        """
        Starts the timer.
        """
        if self.paused:
            self.start_t = time.time()
        self.paused = False

    def pause(self) -> None:
        """
        Pauses the timer. Also updates the time elapsed.
        """
        if not self.paused:
            self.t += time.time() - self.start_t
        self.paused = True
    
    def check(self) -> float:
        """
        Returns the time elapsed.
        """
        self.pause()
        t = self.t
        self.start()
        return t