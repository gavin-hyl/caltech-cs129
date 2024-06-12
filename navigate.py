"""
navigate.py

This script navigates the robot through the course, consulting the user for input
at all intersections.
Team skeletons.
"""

from subsystems.movement import Movement, LINE_FOLLOW_T
from subsystems.odometry import Odometry
from subsystems.graph import STREET_T, Node
import traceback
from shared import SharedData, DECISION_STATUS
import time
from threading import Thread
from subsystems.graph import Graph

class Navigator:
    """
    The Navigator class contains logic for navigating the robot through the course.

    Attributes:
        shared (SharedData): The shared data between threads.
        movement (Movement): The movement subsystem.
        graph_thread (Thread): The thread for handling the graph.
        skip_line_follow (bool): Whether to skip line following.
        odometry (Odometry): The odometry object.
        explore_blocked (bool): Whether to explore blocked nodes (clear them on next explore).
        dist_buf (tuple[float, float, float]): The buffer for the ultrasound sensor readings.
    """

    def __init__(self, shared: SharedData) -> None:
        """
        Initializes the Navigator object. Starts the visualization thread.
        """
        self.shared: SharedData = shared
        self.movement: Movement = Movement(shared.io, shared=shared)
        self.graph_thread: Thread = Thread(name="[graph]", target=self.graph_handler, args=())
        self.graph_thread.start()
        self.skip_line_follow: bool = False
        self.odometry: Odometry = Odometry()
        self.explore_blocked: bool = False
        self.dist_buf = None
        self.init_pose_set = False


    def show(self):
        with self.shared:
            self.shared.show = True


    def navigate(self) -> None:
        """
        Navigates the robot through the course. We assume that we start on a
        connected street.
        """
        def update_ros():
            with self.shared:
                self.shared.robotx, self.shared.roboty, self.shared.robotheading = self.odometry.pos_xyh()
        try:
            line_follow_prev = LINE_FOLLOW_T.UNKNOWN
            exists_line = False
            first = True
            while not self.shared.quit:
                if self.init_pose_set:
                    update_ros()
                if not self.skip_line_follow:
                    blip_avoid_t = 1.5
                    if line_follow_prev == LINE_FOLLOW_T.EOL:
                        blip_avoid_t = 0
                    if first:
                        blip_avoid_t = 0.2
                    self.line_follow_result, exists_line = self.line_follow_handler(blip_avoid_t=blip_avoid_t)
                    while not self.init_pose_set:
                        pass
                update_ros()
                if self.line_follow_result == LINE_FOLLOW_T.EOL:
                    self.u_turn_handler()
                    # line_follow_prev = LINE_FOLLOW_T.EOL
                    # continue
                elif self.line_follow_result in [LINE_FOLLOW_T.INTERSECT, LINE_FOLLOW_T.T_INTERSECT]:
                    is_tunnel = (self.line_follow_result == LINE_FOLLOW_T.T_INTERSECT)
                    if not (line_follow_prev in [LINE_FOLLOW_T.EOL, LINE_FOLLOW_T.UNKNOWN] or self.skip_line_follow):
                        self.update_curr_street(STREET_T.CONNECTED, is_tunnel=is_tunnel)
                        self.odometry.calc_move()
                        self.update_curr_street(STREET_T.CONNECTED, back=True, is_tunnel=is_tunnel)
                    self.update_curr_street(STREET_T.UNEXPLORED if (exists_line or is_tunnel) else STREET_T.NONEXISTENT)
                    self.update_curr_blocked()
                    self.decision()
                elif self.line_follow_result == LINE_FOLLOW_T.BLOCK and line_follow_prev != LINE_FOLLOW_T.BLOCK:
                    if line_follow_prev != LINE_FOLLOW_T.EOL:
                        self.u_turn_handler(STREET_T.UNKNOWN)
                    self.shared.step = True
                    _, exists_line = self.line_follow_handler(prev_obstacle=True, blip_avoid_t=0.3)
                    self.update_curr_street(STREET_T.UNEXPLORED if exists_line else STREET_T.NONEXISTENT)
                    self.update_curr_blocked()
                    self.decision()
                else:
                    self.decision()
                update_ros()
                line_follow_prev = self.line_follow_result
                first = False
        except KeyboardInterrupt:
            print("[navigator] keyboard interrupt")
        except:
            print("[navigator] unknown error")
            print(traceback.format_exc())
        finally:
            self.movement.stop()


# ========================= DECISION-MAKING FUNCTIONS ==========================

    def decision(self) -> None:
        """
        Handles decision-making logic at an intersection. The robot will pause
        until pause is set to False or step is set to True. The robot will then
        check the decision status, and act accordingly. There are three possible
        decision statuses: GOAL, EXPLORE, and MANUAL.
        """
        self.show()
        time.sleep(0.15)
        self.shared.pause_step_block()
        path_find = False
        explore = False
        goal = None
        with self.shared:
            path_find = self.shared.decision_status == DECISION_STATUS.GOAL \
                        and (goal := self.shared.goal) is not None
            explore = self.shared.decision_status == DECISION_STATUS.EXPLORE and not path_find

        took_action = False
        if path_find:
            took_action = self.goal_handler(goal)
            with self.shared:
                self.shared.step = not took_action
        elif explore:
            took_action = self.explore_handler()
            with self.shared:
                self.shared.step = not took_action
        else:
            self.manual_handler(self.shared.direction)
        self.update_curr_blocked()
        self.skip_line_follow = self.skip_line_follow or not self.valid_line_follow()
        self.show()


    def halt_decisions(self, clear_planner: bool = True) -> None:
        """
        Halts the decision-making process. Sets the robot to manual mode, but
        does not allow the robot to move.

        Args:
            clear_planner: whether to clear the planner

        Returns:
            None
        """
        if clear_planner:
            self.shared.graph.reset_planner()
        self.shared.decision_status = DECISION_STATUS.MANUAL
        self.shared.pause = True
        self.shared.step = False
        self.skip_line_follow = True
        self.shared.direction = 0   # default to going forward, but won't execute.


    def manual_handler(self, direction: int) -> None:
        """
        Handles a manual action. After the action is taken, the robot will halt
        decision-making.

        Args:
            direction: the direction to turn (0 for forward)

        Returns:
            None
        """
        if self.line_follow_result != LINE_FOLLOW_T.T_INTERSECT:
            self.blind_turn_handler(direction)
        self.skip_line_follow = direction in [-1, 1]
        with self.shared:
            self.shared.decision_status = DECISION_STATUS.MANUAL
            self.shared.pause = True
            self.shared.step = not self.skip_line_follow


    def goal_handler(self, goal: tuple[int, int]) -> bool:
        """
        Handles goal logic. First checks whether a path to the goal exists, and
        whether the robot is at the goal. If the robot is at the goal, it
        will halt decision-making. Otherwise, we will drive towards the goal.

        Args:
            goal: the goal coordinates

        Returns:
            True if took action, False otherwise
        """
        node = self.this_node()

        # check whether we are at the goal node or not
        with self.shared:
            # replan at decision to ensure the latest information is handled
            self.shared.graph.directed_explore(goal[0], goal[1], curr_pos=self.odometry)
            goal_node = self.shared.graph.get_node(goal[0], goal[1], create=False)
            if (goal_node is not None) and (goal_node == node):
                self.halt_decisions()
                print("[goal] reached goal, clearing planner")
                return True
            
        if self.line_follow_result == LINE_FOLLOW_T.T_INTERSECT:
            self.update_curr_blocked()
            if self.valid_line_follow():
                self.skip_line_follow = False
                return True
            return False
        
        def attempt_plan():
            """ returns whether a path is found """
            with self.shared:
                # replan around blockages. This takes care of both the case
                # where there is a direct path to the goal and the case
                # where no unblocked path exists, in which case we should explore.
                print("[nav] goal-follow/directed explore")
                self.shared.graph.directed_explore(goal[0], goal[1], curr_pos=self.odometry)
                if self.this_node().direction is None:
                    # We have exhausted all unblocked options. Try to recheck.
                    # We only preserve the blocks at the current node and direction.
                    print("[nav] goal-follow/directed explore, ignoring blocks")
                    self.shared.graph.directed_explore(goal[0], goal[1], ignore_blocks=True, curr_pos=self.odometry)
                if self.this_node().direction is None or node.is_blocked[node.direction]:
                    # As a last resort, nuke the blocks and try again
                    print("[nav] goal-follow/directed explore, NUKING BLOCKS")
                    self.shared.graph.clear()
                    self.shared.graph.directed_explore(goal[0], goal[1], curr_pos=self.odometry)
                if self.this_node().direction is None:
                    print("[goal] cannot find path to goal.")
                    self.halt_decisions()
                    return False
            return True

        if not attempt_plan():
            self.skip_line_follow = True
            return False
        dir = node.direction
        self.skip_line_follow = dir != self.odometry.h and node.streets[dir] != STREET_T.NONEXISTENT
        return self.directed_turn_handler(dir)


    def explore_handler(self) -> bool:
        """
        Handles exploration logic. First checks whether there are any incomplete
        nodes in the graph. If there are, it attempts to direct the robot to
        the next incomplete node. If there are no incomplete nodes, the robot
        will halt decision-making.

        Args:
            None

        Returns:
            True if took action, False otherwise
        """
        incomplete_node = None
        if self.explore_blocked:
            with self.shared:
                self.shared.graph.clear()
        with self.shared:
            incomplete_node = self.shared.graph.explore(curr_pos=self.odometry)
        
        if self.line_follow_result == LINE_FOLLOW_T.T_INTERSECT:
            self.update_curr_blocked()
            if self.valid_line_follow():
                self.skip_line_follow = False
                return True
            return False

        node = self.this_node()
        masked_streets = node.mask_blocked()
        if node.direction is not None:
            took_action = self.directed_turn_handler(node.direction)
            self.explore_blocked = False
            self.skip_line_follow = (bool(set(masked_streets) & Graph.EXPLORABLE)) \
                                     and not masked_streets[self.odometry.h] == STREET_T.UNKNOWN \
                                     and masked_streets[node.direction] in [STREET_T.EOL, STREET_T.NONEXISTENT, STREET_T.UNKNOWN]
            if incomplete_node is None:
                self.halt_decisions()
        else:
            took_action = True
            self.skip_line_follow = True
            self.explore_blocked = True
            self.halt_decisions()
        return took_action


# ====================== MOVEMENT HELPER FUNCTIONS =============================

    def u_turn_handler(self, street_status=STREET_T.EOL) -> None:
        """
        Handles a u-turn. Updates the current node's street statuses, as
        well as the robot's heading.

        Args:
            None

        Returns:
            None
        """
        self.update_curr_street(street_status)
        self.movement.turn_to_next(1)
        self.odometry.calc_u_turn()


    def line_follow_handler(self,
                            prev_obstacle: bool = False,
                            blip_avoid_t: float = 0) -> tuple[int, int]:
        """
        Handles line following.

        Args:
            prev_obstacle: whether the previous line_follow was a block

        Returns:
            The type of ending line_follow encountered, and whether a line exists
        """
        self.show()
        # block control flow until we have steps
        self.shared.pause_step_block()
        INF = 1e6
        block_wait_t = 3 + prev_obstacle * INF
        if not prev_obstacle:
            self.update_curr_blocked()
        if self.odometry.h % 2 == 1:
            blip_avoid_t *= 1.5
        lf_result = self.movement.line_follow(block_wait_t, blip_avoid_t)
        self.dist_buf = self.shared.distances
        if lf_result != LINE_FOLLOW_T.BLOCK:
            exists_line = self.movement.pull_forward()
            return lf_result, exists_line
        else:
            return lf_result, True


    def blind_turn_handler(self, direction: int) -> None:
        """
        Handles a blind turn. Updates the current node's street statuses, as
        well as the robot's heading.

        Args:
            direction: the direction to turn

        Returns:
            None
        """
        self.update_curr_blocked()
        if direction == 0 or self.line_follow_result == LINE_FOLLOW_T.T_INTERSECT:  # no turn
            return
        turn_index = self.movement.turn_to_next(direction)
        h_now = self.odometry.h

        h_end = self.odometry.h = self.validate_turn(1, direction, (h_now + turn_index)%8)
        if self.detect_lost_status((h_now + turn_index)%8, h_end):
            return
        if not h_end == h_now:
            while True:
                h_now = (h_now + direction) % 8
                if Odometry.h_diff(h_now, h_end) == 0:
                    break
                self.update_curr_street(STREET_T.NONEXISTENT, h=h_now)
        self.update_curr_street(STREET_T.UNEXPLORED)
        self.update_curr_blocked()


    def directed_turn_handler(self, h_desired: int) -> None:
        """
        Handles a directed turn. Updates the current node's street statuses, as
        well as the robot's heading. Validates the information returned
        by turn_to_heading with the map. Also updates the blocked status of the
        node.

        Args:
            h_desired: the desired heading

        Returns:
            None
        """
        h_now = self.odometry.h
        self.update_curr_blocked()
        if Odometry.h_diff(h_now, h_desired) == 0 or self.line_follow_result == LINE_FOLLOW_T.T_INTERSECT:
            return False
        existing_streets, direction = self.movement.turn_to_heading(h_now, h_desired)
        end_street = existing_streets[-1]
        self.odometry.h = self.validate_turn(len(existing_streets), direction, end_street)
        if self.detect_lost_status(self.odometry.h, end_street):
            return True
        if (self.odometry.h == end_street) or len(existing_streets) == 1:
            while True:
                h_now = (h_now + direction) % 8
                if Odometry.h_diff(h_now, self.odometry.h) == 0:
                    break
                street_status = STREET_T.UNEXPLORED if h_now in existing_streets else STREET_T.NONEXISTENT
                self.update_curr_street(street_status, h=h_now)
        else:
            # The h of the robot does not agree with the angle sensor readings.
            # In this case, if there are multiple streets, then we don't want to
            # do anything except update the very last street.
            pass
        self.update_curr_street(STREET_T.UNEXPLORED)
        self.update_curr_blocked()
        return True


    def validate_turn(self, turns_made: int, direction: int, end: int) -> int:
        """
        Validates a turn using the map. Updates the odometry object. This method
        iterates through the streets in the direction of the turn, and checks
        their statuses. If no streets are UNKNOWN, we can count the number of
        statuses that are not NONEXISTENT, and determine the new heading of the
        robot. Otherwise, the turn is (possibly) valid, since we do not have any
        information regarding that street. We can then check the end status of
        the street, and validate the turn that way.

        Args:
            turns_made: the number of turns made
            direction: the direction turned
            end: the ending index of the turn, returned by AngleSensor

        Returns:
            What the theoretical new heading should be.
        """
        h_now = self.odometry.h
        node = self.this_node()
        turn_cnt = 0
        for _ in range(8):
            h_now = (h_now + direction) % 8
            street = node.streets[h_now]
            if street == STREET_T.UNKNOWN:
                break   # we cannot hope to use the iterative method, so we check directly the end status
            elif street != STREET_T.NONEXISTENT:
                 turn_cnt += 1
            if turns_made == turn_cnt:
                # maybe the end was correct
                return h_now

        if node.streets[end] != STREET_T.NONEXISTENT:
            return end

        pos_end = (end + direction) % 8
        neg_end = (end - direction) % 8
        check_sequence = [pos_end, neg_end]
        if abs(Odometry.h_abs_cap(Odometry.h_diff(self.odometry.h, end))) >= 2:
            check_sequence.reverse()
        for h_end in check_sequence:
            if node.streets[h_end] != STREET_T.NONEXISTENT:
                return h_end

        with self.shared:
            self.shared.lost = True
            self.halt_decisions()
        return end # should never get here


    def detect_lost_status(self, measured_end, validated_end) -> bool:
        """
        Detects if the robot is lost by checking whether the difference between
        the measured end street and the validated end street have a discrepancy
        greater than two. If lost, sets the lost flag in shared data to true.

        Args:
            measured_end: the ending index of the turn, returned by AngleSensor
            validated_end: the ending index of the turn, as calculated by validate_turn

        Return:
            A boolean value reflecting the lost status.
        """
        h_diff = Odometry.h_abs_cap(Odometry.h_diff(measured_end, validated_end))
        if abs(h_diff) >= 2:
            with self.shared:
                self.shared.lost = True
            return True
        return False
        # diff = abs(validated_end - measured_end)
        # sign = 1 if diff < 4 else -1
        # if (sign * diff % 8) >= 2:
        #     with self.shared:
        #         self.shared.lost = True
        #     return True
        # return False


# ========================= GRAPH HELPER FUNCTIONS =============================

    def update_curr_street(self,
                           status:int,
                           back: bool=False,
                           h: int=None,
                           is_tunnel: bool=False) -> None:
        """
        Updates a street at the current location in the map.

        Arguments:
            status: the status to be updated
            back: whether or not to reverse the heading before updating, does
                  not change the odometry object
            h: the heading of the street to update, defaults to current heading
            is_tunnel: whether or not the street is a tunnel

        Returns:
            None
        """
        if not self.init_pose_set:
            return
        x, y = self.odometry.pos_xy()
        if h is None:
            h = self.odometry.h
        if back:
            h = Odometry.h_opposite(h)
        with self.shared:
            self.shared.graph.update_street(x, y, h, status)
            if is_tunnel:
                for i in range(1,4):
                    self.shared.graph.update_street(x, y, h+i, STREET_T.NONEXISTENT)
                    self.shared.graph.update_street(x, y, h-i, STREET_T.NONEXISTENT)


    def update_curr_blocked(self, thresh: float = 0.6) -> None:
        """
        Updates the current location in the map with the ultrasound sensors.

        Arguments:
            thresh: threshold for the proximity sensor to recognize a blockage

        Returns:
            None
        """
        if not self.init_pose_set:
            return
        x, y, h = self.odometry.pos_xyh()
        if h % 2 == 1:
            thresh *= 2**0.5
        prox_readings = self.read_dist()
        blocked_status = [dist < thresh for dist in prox_readings]
        with self.shared:
            # self.shared.graph.update_block(x, y, h+2, blocked_status[0])
            self.shared.graph.update_block(x, y, h  , blocked_status[1])
            # self.shared.graph.update_block(x, y, h-2, blocked_status[2])


    def this_node(self) -> Node:
        """
        Returns the current node. Read-only reference to the graph.
        """
        x, y = self.odometry.pos_xy()
        return self.shared.graph.get_node(x, y, create=False)


    def valid_line_follow(self) -> bool:
        """ Returns True if going straight is valid, False otherwise. """
        node = self.this_node()
        h = self.odometry.h
        INVALID_LF_STREETS = [STREET_T.NONEXISTENT, STREET_T.EOL, STREET_T.UNKNOWN]
        return node.mask_blocked()[h] not in INVALID_LF_STREETS


    def graph_handler(self) -> None:
        """
        Visualizes the robot's current position and the graph.

        Args:
            odometry: the Odometry object
            shared: the SharedData object
        """
        shared = self.shared
        try:
            while not shared.quit:
                with shared:
                    if shared.clear:
                        shared.graph.clear()
                        shared.clear = False
                    if shared.save:
                        shared.graph.save_pk()
                        shared.save = False
                    if shared.show:
                        shared.graph.save_png(self.odometry)
                        shared.show = False
                    if shared.pose:
                        buf = shared.pose_buf
                        self.odometry.x = buf[0]
                        self.odometry.y = buf[1]
                        self.odometry.h = buf[2]
                        self.init_pose_set = True
                        shared.pose = False
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("[vis] keyboard interrupt")


# ============================== MISC FUNCTIONs ================================
    def read_dist(self) -> tuple[float, float, float]:
        """
        Reads the ultrasound sensor distances and updates the shared distances.
        """
        if (dists := self.dist_buf) is not None:
            self.dist_buf = None
            return dists
        with self.shared:
            return self.shared.distances


def main(shared: SharedData):
    """
    Main function for the navigator. Initializes the navigator and starts the
    navigation process.
    """
    with shared.lock:
        navigator = Navigator(shared)
    try:
        navigator.navigate()
        navigator.graph_thread.join()
    except:
        print("oh no")
        navigator.movement.stop()