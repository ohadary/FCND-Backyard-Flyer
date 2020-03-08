import argparse
import time
from enum import Enum
import math
import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL    = 0
    ARMING    = 1
    TAKEOFF   = 2
    WAYPOINT  = 3
    LANDING   = 4
    DISARMING = 5

def print_debug(msg):
    if DEBUG: print(msg)

class BackyardFlyer(Drone):

    def __init__(self, connection, box_width = 10.0, box_length = 10.0, flying_altitude = 3):
        super().__init__(connection)

        self.in_mission = True

        # initial state
        self.flight_state = States.MANUAL

        # box dimensions
        self.box_width  = box_width
        self.box_length = box_length

        # target flying altitude
        self.flying_altitude = flying_altitude

        # obtain trip waypoints
        self.all_waypoints = self.calculate_box()

        # initialize target position to first trip waypoint
        self.target_position = self.all_waypoints.pop(0)

        self.check_state = dict({States.WAYPOINT : self.traverse_waypoints,
                                 States.LANDING  : self.is_safe_to_disarm})

        self.safe_to_disarm = False

        # Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE,          self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        for state,action in self.check_state.items():
            if self.flight_state == state:
                action()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        for state,action in self.check_state.items():
            if self.flight_state == state:
                action()

    def is_safe_to_disarm(self):
        self.safe_to_disarm = True if self.local_position[2] <= 0.05 and np.linalg.norm(self.local_velocity) < 0.1 else False

    def advance_waypoint(self):
        """
        If drone is 'loitering' in vicinity of 'waypoint' advance to next waypoint and return True, o/w return False
        """
        if self.target_position is None:
            return False

        # check position constraint
        if abs(np.linalg.norm(self.target_position) - np.linalg.norm(self.local_position[0:3])) < 0.5:
            # check velocity constraint
            if abs(np.linalg.norm(self.local_velocity)) < 0.2:
                print_debug('waypoint reached: {}'.format(self.target_position))
                self.target_position = self.all_waypoints.pop(0) if len(self.all_waypoints) else None
                return True
        return False

    def traverse_waypoints(self):
        """
        Go to the next waypoint if drone is at target waypoint
        """
        self.advance_waypoint()

    def execute_landing(self):
        return self.target_position is None

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
         guided : client connected to simulator for autonomous (programatic) control
         armed  : motors are powered and quad is ready to fly
        """

        if self.flight_state == States.MANUAL:
            self.arming_transition()

        elif self.flight_state == States.ARMING:
            self.takeoff_transition()

        elif self.flight_state == States.TAKEOFF:
            self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:
            self.landing_transition() if self.execute_landing() else self.waypoint_transition()

        elif self.flight_state == States.LANDING:
            # make sure we're sufficiently close to the ground before disarming drone
            if self.safe_to_disarm:
                self.disarming_transition()

        elif self.flight_state == States.DISARMING:
            self.manual_transition()

        else:
            print_debug('Guided: {} Armed: {}'.format(self.guided, self.armed))

    def calculate_box(self):
        """TODO: Fill out this method

        1. Return trip waypoints
        """
        n,e,d = self.local_position
        l, w, h = self.box_length, self.box_width, self.flying_altitude

        return [np.array([n    , e    , d + h, 0]),
                np.array([n + l, e    , d + h, 0]),
                np.array([n + l, e + w, d + h, 0]),
                np.array([n    , e + w, d + h, 0]),
                np.array([n    , e    , d + h, 0])]

    def arming_transition(self):
        """ Transitions drone into the ARMING (ARMED) state
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print_debug("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(*self.global_position)
        global starting_coordinates
        starting_coordinates = self.local_position
        print('starting coordinates: {}'.format(starting_coordinates))
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method

        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print_debug("takeoff transition")

        # The first target waypoint is the starting local coordinate translated to the target
        # flying altitude
        self.takeoff(self.target_position[2])

        # Transition to TAKEOFF state once we're at the target altitude
        if self.advance_waypoint():
            self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method

        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        self.cmd_position(*self.target_position)
        if not self.flight_state == States.WAYPOINT:
            print_debug("waypoint transition")
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method

        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print_debug("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method

        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print_debug("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided

        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print_debug("manual transition")

        self.release_control()
        final_coordinates = self.local_position
        print('final coordinates: {}'.format(final_coordinates))
        global starting_coordinates
        print('roundtrip displacement vector norm: {}'.format(np.linalg.norm(starting_coordinates - final_coordinates)))
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided

        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print_debug("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print_debug("starting connection")
        self.connection.start()
        print_debug("Closing log file")
        self.stop_log()

    def position(self):
        print_debug('local  {}'.format(self.local_position))
        print_debug('global {}'.format(self.global_position))
        print_debug('home   {}'.format(self.global_home))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',     type=int,   default=5760,        help='Port number')
    parser.add_argument('--host',     type=str,   default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--width',    type=float, default=10.0,        help="flight box width")
    parser.add_argument('--length',   type=float, default=10.0,        help="flight box length")
    parser.add_argument('--altitude', type=float, default=10.0,        help="flying altitude")
    parser.add_argument('--debug', action="store_true", default=False, help="print debug messages")
    args = parser.parse_args()
    global DEBUG
    DEBUG = args.debug

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn, args.width, args.length, args.altitude)
    time.sleep(2)
    drone.start()
