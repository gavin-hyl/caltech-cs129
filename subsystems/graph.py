"""
graph.py

Contains the Node and Graph classes, which contain logic for the robot's
recording and visualization of the track.

Team skeletons.
"""

import matplotlib.pyplot as plt
import pickle
from pathlib import Path
from enum import Enum
from subsystems.odometry import Odometry
from copy import deepcopy
import numpy as np



class STREET_T(Enum):
    UNKNOWN = 0
    NONEXISTENT = 1
    UNEXPLORED = 2
    EOL = 3
    CONNECTED = 4



class Node:
    """
    Represents a node in the graph. Each node has a position, a list of
    street statuses, a direction, and a cost. The direction is the optimal
    path to the goal, and the cost is the cost of the optimal path to the goal.

    Attributes:
        x: the x position of the node.
        y: the y position of the node.
        streets: a list of statuses for each street (0-7).
        direction: the direction of the optimal path to the goal.
        optimized: a boolean indicating whether the node has been optimized.
        cost: the cost of the optimal path to the goal.
        is_blocked: a list of blocked statuses, True if blocked, False otherwise
    """
    def __init__(self, x, y):
        """
        Initializes the node object. x and y denote the position of the
        node relative to the robot's starting point.
        """
        self.x: int = x
        self.y: int = y
        self.streets: list[STREET_T] = [STREET_T.UNKNOWN] * 8
        self.is_blocked: list[bool] = [False] * 8
        self.direction: int = None
        self.optimized: bool = False
        self.cost: float = float('inf')
        self.reset()

    def reset(self):
        """ Resets the node to its default state (excluding streets) """
        self.direction = None
        self.optimized = False
        self.cost = float('inf')

    def clear_blocked(self, h: int = None):
        if h is None:
            self.is_blocked = [False] * 8
        else:
            self.is_blocked[h] = False

    def mask_blocked(self) -> list[int]:
        masked_streets = []
        for street, blocked in zip(self.streets, self.is_blocked):
            masked_streets.append(STREET_T.EOL if blocked else street)
        return masked_streets


    def pos(self) -> tuple[int, int]:
        return self.x, self.y


    def __eq__(self, other):
        if other is None:
            return False
        return self.x == other.x and self.y == other.y


    def __str__(self):
        return f"({self.x}, {self.y}): cost={self.cost}, direction={self.direction}"

    def __repr__(self):
        return self.__str__()



class Graph:
    """
    Abstraction of a graph (map), which is essentially a dictionary of
    nodes. The graph is defined by n nodes with each node having 8
    possible streets.

    Attributes:
        nodes: the dictionary of nodes keyed with a position tuple.
        goal: the goal node of the graph.
        _save_path: the path to save the graph to.
    """

    _ALLOWED_UPDATES = {
        STREET_T.UNKNOWN: set([STREET_T.UNEXPLORED, STREET_T.NONEXISTENT, STREET_T.EOL, STREET_T.CONNECTED]),
        STREET_T.NONEXISTENT: set(),
        STREET_T.UNEXPLORED: set([STREET_T.CONNECTED, STREET_T.EOL]),
        STREET_T.EOL: set(),
        STREET_T.CONNECTED: set()
    }
    _SAVE_DIR = Path.cwd() / "maps"
    EXPLORABLE = set([STREET_T.UNEXPLORED, STREET_T.UNKNOWN])

    def __init__(self):
        self.nodes = {}
        self.goal = None
        self._save_path = Path(__file__).resolve().parent / "maps"


    def get_node(self, x: int, y: int, create: bool = True) -> Node:
        """
        Grabs the specified node. If the node hasn't been used before,
        it creates a new one.

        Arguments:
            x: the x unit coordinate of the node
            y: the y unit coordinate of the node

        Return:
            the node at the given node
        """
        if (x, y) not in self.nodes and create:
            self.nodes[(x, y)] = Node(x, y)
        return self.nodes.get((x, y), None)


    def update_street(self, x: int, y: int, h: int, status: int):
        """
        Updates the node dictionary.

        Arguments:
            x: the x unit coordinate of the node
            y: the y unit coordinate of the node
            heading: the direction (index) of the street (from 0 to 7)
            status: one of the statuses from the enum

            Return:
                the updated nodes dictionary
        """
        h %= 8
        node = self.get_node(x, y)
        if status in Graph._ALLOWED_UPDATES[node.streets[h]]:
            node.streets[h] = status
        self.remove_adjacent([node])


    def update_block(self, x: int, y: int, h: int, status: bool) -> None:
        """
        Updates the node dictionary.

        Arguments:
            x: the x unit coordinate of the node
            y: the y unit coordinate of the node
            heading: the direction (index) of the street (from 0 to 7)
            status: the blocked status of the street

            Return:
                the updated nodes dictionary
        """
        h %= 8
        self.get_node(x, y).is_blocked[h] = status
        dx, dy = Odometry.h_dxy(h)
        if self.get_node(x+dx, y+dy, create=False) is not None:
            self.get_node(x+dx, y+dy, create=False).is_blocked[Odometry.h_opposite(h)] = status


    def explore(self, curr_pos: Odometry) -> Node:
        """
        Plans a path to the nearest unexplored/unknown node, and updates
        the direction and cost attributes of each node accordingly. We prioritize
        UNEXPLORED over UNKNOWN.

        Args:
            curr_pos: the current position of the robot

        Returns:
            The nearest explorable node. None if fully explored.
        """
        self.reset_planner()
        x, y, h = curr_pos.pos_xyh()
        self.plan_path(x, y)

        targets, target_blocked = self.get_unexplored()

        if len(targets) == 0 or target_blocked:
            self.reset_planner()
            print("[graph] no unexplored nodes found")
            return None

        target = min(targets, key=lambda node: node.cost)
        self.plan_path(target.x, target.y)

        # we only bother with the target heading if we're at the node
        if (x, y) == (target.x, target.y):
            self.set_target_direction(h, target, target_blocked)
        return target


    def directed_explore(self,
                         x_goal: int,
                         y_goal: int,
                         curr_pos: Odometry,
                         ignore_blocks: bool = False) -> Node:
        """
        Attempts to find the shortest path to the goal node. If the goal node
        is not in the graph, it will attempt to find the nearest unexplored node
        and plan a path to that node.

        Args:
            x_goal: the x position of the goal node
            y_goal: the y position of the goal node
            curr_pos: the current position of the robot

        Returns:
            The node that is the nearest explorable node. None if fully explored.
        """
        x, y, h = curr_pos.pos_xyh()
        curr = self.get_node(x, y, create=False)
        if self.plan_path(x_goal, y_goal, ignore_blocks=ignore_blocks) and curr.direction is not None:
            return self.goal

        targets, _ = self.get_unexplored(ignore_blocks=ignore_blocks)
        if len(targets) == 0:
            self.reset_planner()
            print("[graph] no unexplored nodes found")
            return None

        # first set the node costs of the targets
        self.plan_path(x, y, ignore_blocks=ignore_blocks)
        # then adjust them based on the goal
        for node in targets:
            node.cost += ((node.x - x_goal) ** 2 + (node.y - y_goal) ** 2) ** 0.5
        # goal-follow to the target with the lowest cost
        target = min(targets, key=lambda node: node.cost)
        self.plan_path(target.x, target.y, ignore_blocks=ignore_blocks)

        if (x, y) == (target.x, target.y):
            self.reset_planner()
            h_to_goal = Odometry.h_to(x, y, x_goal, y_goal)
            for diff in range(5):
                dir = np.sign(Odometry.h_abs_cap(Odometry.h_diff(h_to_goal, h)))
                dir = 1 if dir == 0 else dir
                h_end_pos = (h_to_goal + diff * dir) % 8
                h_end_neg = (h_to_goal - diff * dir) % 8
                for h_end in [h_end_neg, h_end_pos]:
                    if target.mask_blocked()[h_end] in Graph.EXPLORABLE:
                        target.direction = h_end
                        return target
        return target


    def plan_path(self,
                  x_goal: int,
                  y_goal: int,
                  ignore_blocks: bool = False,
                  curr_pos: Odometry = None) -> None:
        """
        An implementation of Dijkstra's algorithm to find the shortest path from
        all nodes to the goal node. This method updates the cost and direction
        attributes of each node, and returns nothing.

        Args:
            x_goal: the x position of the goal node
            y_goal: the y position of the goal node
            ignore_blocks: whether or not to ignore blocked nodes
            curr_pos: the current position of the robot. The blockages are
                      preserved at this point. We assume that the robot is
                      facing a blockage.

        Returns:
            Whether or not a path to the goal node can be found (i.e., whether
            the goal node exists in the graph)
        """
        self.reset_planner()
        if (goal_node := self.get_node(x_goal, y_goal, create=False)) is None:
            return False

        self.goal = goal_node
        goal_node.cost = 0
        goal_node.direction = None
        frontier = [goal_node]

        if curr_pos is not None:
            curr_node = self.get_node(curr_pos.x, curr_pos.y)
            dx, dy = Odometry.h_dxy(curr_pos.h)
            curr_node_opp = self.get_node(curr_pos.x + dx, curr_pos.y + dy, create=False)

        while len(frontier) > 0:
            working_node = frontier.pop(0)
            working_node.optimized = True

            if ignore_blocks and (curr_pos is None or working_node not in [curr_node, curr_node_opp]):
                working_streets = working_node.streets
            elif curr_pos is not None and working_node == curr_node:
                working_streets = deepcopy(working_node.streets)
                working_streets[curr_pos.h] = STREET_T.NONEXISTENT
            elif curr_pos is not None and working_node == curr_node_opp:
                working_streets = deepcopy(working_node.streets)
                working_streets[Odometry.h_opposite(curr_pos.h)] = STREET_T.NONEXISTENT
            else:
                working_streets = working_node.mask_blocked()

            for direction, street in enumerate(working_streets):
                # iterate through all connected (leaf) nodes
                if street == STREET_T.CONNECTED:
                    dx, dy = Odometry.h_dxy(direction)
                    leaf_node = self.get_node(working_node.x + dx, working_node.y + dy)
                    new_cost = round(working_node.cost + (dx**2 + dy**2) ** 0.5, 1)
                    if leaf_node.optimized or leaf_node.cost <= new_cost:
                        continue
                    leaf_node.cost = new_cost
                    leaf_node.direction = Odometry.h_opposite(direction)
                    inserted = False
                    for index, node in enumerate(frontier):
                        if leaf_node.cost < node.cost and not inserted:
                            frontier.insert(index, leaf_node)
                            inserted = True
                        if leaf_node == node:
                            if inserted:
                                frontier.pop(index)
                            break
                    if len(frontier) == 0 or not inserted:
                        frontier.append(leaf_node)
        return True


    def get_unexplored(self, ignore_blocks: bool = False) -> list[Node]:
        """
        Returns a list of unexplored nodes.
        """
        if ignore_blocks:
            unexplored = []
            for node in self.nodes.values():
                if set(node.streets) & Graph.EXPLORABLE:
                    unexplored.append(node)
            return unexplored, None

        unblocked_explorables = []
        blocked_explorables = []
        has_unblocked = False
        for node in self.nodes.values():
            for street in node.mask_blocked():
                if street in Graph.EXPLORABLE:
                # if set(node.mask_blocked()) & Graph.EXPLORABLE:
                    unblocked_explorables.append(node)
                    has_unblocked = True
                    break
            # we only allow for UNKNOWN and blocked
            # to solve the problem of turning to a UNEXPLORED and rejecting
            # the line-follow, which causes an infinite loop
            if not has_unblocked and STREET_T.UNKNOWN in node.streets:
                blocked_explorables.append(node)
        targets = unblocked_explorables if has_unblocked else blocked_explorables
        return targets, not has_unblocked


    def set_target_direction(self,
                             h: int,
                             target: Node,
                             target_blocked: bool) -> None:
        # clear all other nodes except this one
        self.reset_planner()
        target_street_status = STREET_T.UNKNOWN
        # calculating which street status to turn to, prioritizing UNEXPLORED
        # which we only consider if some streets are not blocked
        if not target_blocked and (STREET_T.UNEXPLORED in target.mask_blocked()):
            target_street_status = STREET_T.UNEXPLORED

        # calculating which street index to turn to
        for diff in range(5):
            h_end_pos = (h + diff) % 8
            h_end_neg = (h - diff) % 8
            for h_end in [h_end_pos, h_end_neg]:
                if (target.mask_blocked()[h_end] == target_street_status == STREET_T.UNEXPLORED) \
                    or (target.streets[h_end] == target_street_status == STREET_T.UNKNOWN):
                    target.direction = h_end
                    return
        print("[graph] no suitable direction found")
        target.direction = None


    def remove_adjacent(self,
                        nodes: list[Node] = None) -> None:
        """ Removes adjacent streets of existing streets. """
        EXISTING = set([STREET_T.CONNECTED, STREET_T.EOL, STREET_T.UNEXPLORED])
        if nodes is None:
            nodes = self.nodes.values()
        for node in nodes:
            new_streets = node.streets
            for idx, status in enumerate(node.streets):
                if status in EXISTING:
                    new_streets[(idx+1) % 8] = STREET_T.NONEXISTENT
                    new_streets[(idx-1) % 8] = STREET_T.NONEXISTENT
            node.streets = new_streets


    def reset_planner(self) -> None:
        """ Resets the attributes of all nodes to their default. """
        for node in self.nodes.values():
            node.reset()


    def clear(self) -> None:
        """ Clears node blockage statuses. """
        for node in self.nodes.values():
            node.clear_blocked()


    def save_png(self, pos: Odometry, arrow:bool=True, name: str = "graph"):
        """
        Saves the graph as a grid of nodes with outgoing nodes.

        Arguments:
            pos: the current position of the robot
            arrow: boolean indicating whether to draw direction-of-travel arrows
            graph_name: the name of the graph being saved
        """
        STREET_T_COLORS = {
            STREET_T.UNKNOWN: "black",
            STREET_T.NONEXISTENT: "lightgray",
            STREET_T.UNEXPLORED: "blue",
            STREET_T.EOL: "red",
            STREET_T.CONNECTED: "green"
        }

        plt.clf()
        plt.axes()
        plt.xlim(-6.5, 6.5)
        plt.ylim(-6.5, 6.5)
        plt.gca().set_aspect('equal', adjustable='box')

        for x in range(-6, 7):
            for y in range(-6, 7):
                plt.plot(x, y, color='lightgray', marker='o', markersize=5)

        for (x, y), node in self.nodes.items():
            for street_heading, street_status in enumerate(node.streets):
                dx, dy = Odometry.h_dxy(street_heading)
                dx *= 0.5
                dy *= 0.5
                status_color = STREET_T_COLORS.get(street_status, 'black')
                plt.plot([x, x + dx], [y, y + dy], color=status_color, lw=1, zorder=3)
                if street_heading == node.direction:

                    plt.arrow(x, y, dx*1.4, dy*1.4, width=0, shape='full',
                              lw=0, length_includes_head=True, head_width=.2,
                              color="black", zorder=1)
            for street_heading, is_blocked in enumerate(node.is_blocked):
                dx, dy = Odometry.h_dxy(street_heading)
                dx *= 0.5
                dy *= 0.5
                if is_blocked:
                    plt.scatter(x+dx, y+dy, c="red", marker="8", zorder=10, s=4)
        x, y, h = pos.pos_xyh()
        dx, dy = Odometry.h_dxy(h)
        if arrow:
            plt.arrow(x, y, dx*0.5, dy*0.5, width=0.1, head_width=0.2,
                    head_length=0.1, color='magenta', fc="none", zorder=2)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.savefig(f"{Graph._SAVE_DIR}/{name}.png")


    def save_pk(self, graph_name: str = None):
        """ Saves the graph as a pickle file. """
        if graph_name is None:
            try:
                map_name = input('Please provide the name of the map.\n>>>')
            except KeyboardInterrupt:
                return
        else:
            map_name = graph_name
        map_name = map_name if len(map_name) > 0 else "default"
        full_path = Graph._SAVE_DIR / f"{map_name}.pickle"
        print(f'saving graph to {full_path}')
        with open(full_path, 'wb') as f:
            pickle.dump(self, f)


    @staticmethod
    def load_pk(map_name: str):
        """
        Loads a graph from the saved pickle file. If the filename is not provided,
        the default file is loaded.
        """
        map_name = map_name if len(map_name) > 0 else "default"
        full_path = Graph._SAVE_DIR / f"{map_name}.pickle"
        try:
            print(f'loading graph from {full_path}')
            with open(full_path, 'rb') as f:
                graph = pickle.load(f)
                graph.reset_planner()
                return map_name, graph
        except FileNotFoundError:
            print("Graph not found, continuing with empty graph.")
            return map_name, None
