"""
UI.py

This file contains the UI code and thread initialization.
"""

import ctypes
from shared import SharedData, DECISION_STATUS
import subsystems.hardware_interface.proximitysensor as prox
import threading
import navigate
from subsystems.graph import Graph
from ros import runros


def main():
    shared = SharedData()
    robot_thread = threading.Thread(name="navigate", target=navigate.main, args=(shared,))
    ultrasound_thread =threading.Thread(name="trigger", target=prox.main, args=(shared,))
    ros_thread = threading.Thread(name="ros", target=runros, args=(shared,))
    robot_thread.start()
    ultrasound_thread.start()
    ros_thread.start()
    set_pose(shared)

    try:
        while not shared.quit:
            UI(shared)
    except KeyboardInterrupt:
        print()
        print("[UI] keyboard interrupt")
        shared.quit = True
    finally:
        kill_thread(robot_thread)
        kill_thread(ultrasound_thread)
        kill_thread(ros_thread)
        robot_thread.join()
        ultrasound_thread.join()
        shared.io.stop()


def kill_thread(thread: threading.Thread):
    """
    This function kills a thread by raising an exception in the thread.
    """
    ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(thread.ident),
        ctypes.py_object(KeyboardInterrupt))


def set_share_attr(shared: SharedData, attr: str, value):
    """
    This function sets the attribute of the shared data object.
    """
    with shared:
        setattr(shared, attr, value)


def set_explore(shared: SharedData):
    """
    This function sets the robot to explore mode.
    """
    with shared:
        shared.decision_status = DECISION_STATUS.EXPLORE
        shared.step = True
        shared.pause = False


def define_goal(shared: SharedData):
    """
    This function sets the goal of the robot.
    """
    x = int(input("Enter the x coordinate of the goal: "))
    y = int(input("Enter the y coordinate of the goal: "))
    goal = (x, y)
    with shared:
        shared.graph.reset_planner()
        shared.goal = goal
        shared.decision_status = DECISION_STATUS.GOAL
        shared.pause = False


def set_manual(shared: SharedData, direction: int=0):
    """
    This function sets the robot to manual mode.
    """
    with shared:
        shared.decision_status = DECISION_STATUS.MANUAL
        shared.direction = direction
        shared.step = True
        shared.pause = True


def save_graph(shared: SharedData):
    """
    This function saves the graph.
    """
    name = input("Enter the name of the file to save the graph to: ")
    if name == "":
        name = "graph"
    with shared:
        shared.graph.save_pk(name)


def load_graph(shared: SharedData):
    """
    This function loads the graph into the shared data object for the robot.
    """
    name = input("Enter the name of the file to load the graph from: ")
    if name == "":
        name = None
    with shared:
        shared.graph = Graph.load_pk(name)[1]


def set_pose(shared: SharedData):
    try:
        x = int(input("Enter the x coordinate of the pose >>> "))
        y = int(input("Enter the y coordinate of the pose >>> "))
        h = int(input("Enter the heading of the robot >>> "))
        with shared:
            shared.pose_buf = (x, y, h)
    except ValueError:
        print("[UI] a value wasn't an integer, defaulting to 0, 0, 0")
        with shared:
            shared.pose_buf = (0, 0, 0)
    with shared:
        shared.pose = True


def lost_alert(shared: SharedData):
    if shared.lost:
        shared.pause = True
        shared.step = False
        print("The robot may be lost, do not resume until robot location is rectified manually!!!")
        set_pose(shared)
        shared.lost = False


CMD_TO_FLAGS = {
    "explore": set_explore,
    "goal": define_goal,

    "pause": lambda shared: set_share_attr(shared, "pause", True),
    "step": lambda shared: set_share_attr(shared, "step", True),
    "resume": lambda shared: set_share_attr(shared, "pause", False),

    "left": lambda shared: set_manual(shared, 1),
    "right": lambda shared: set_manual(shared, -1),
    "forward": lambda shared: set_manual(shared, 0),

    "save": save_graph,
    "load": load_graph,
    "pose": set_pose,

    "show": lambda shared: set_share_attr(shared, "show", True),
    "quit": lambda shared: set_share_attr(shared, "quit", True),
    "clear": lambda shared: set_share_attr(shared, "clear", True),
}

def UI(shared: SharedData):
    commands = list(CMD_TO_FLAGS.keys())
    lost_alert(shared)
    cmd = input(f"enter a command from the list: {commands} \n >>>")
    print(f"Recieved command: {cmd}")
    CMD_TO_FLAGS.get(cmd, lambda _: print("Unknown command."))(shared)
    print("Executed command.")
    print(shared)


if __name__ == "__main__":
    main()