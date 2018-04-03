import argparse
import time
from enum import Enum

import numpy as np

import visdom
from subprocess import call
#import subprocess

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        ######## ------------ Start of Visualization Code ------------------- #########
        # Visdom server - default opens to http://localhost:8097
        # proc = subprocess.Popen('python", "-m", "visdom.server"], shell=False)
        # call(["python", "-m", "visdom.server"])
        # status = subprocess.check_output(['python', '-m', 'visdom.server'])
        # print (status)

        self.v = visdom.Visdom()
        assert self.v.check_connection()

        # Registering Plotting callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.update_ne_plot_callback)
        self.register_callback(MsgID.LOCAL_POSITION, self.update_altitude_plot_callback)

        # plotting North and East Position (NE plot)
        # Note [1] is East and [0] is North, for Plotting purpose, they are switched
        ne  = np.array([self.local_position[1], self.local_position[0]]).reshape(1,-1) # plotting east north
        # ne  = np.array([self.local_position[0], self.local_position[1]]).reshape(1,-1) # plotting north east
        self.ne_plot = self.v.scatter(ne, opts=dict(
            title="Local position (north, east)",
            xlabel = 'East',
            ylabel = 'North'
        ))

        # plotting altitude (D plot)
        # Multiplying by -1.0 just for plotting purposes - to plot up vs. down
        d = -1.0 * np.array([self.local_position[2]]) # plotting up
        #d = np.array([self.local_position[2]]) # plotting down
        self.t = 0
        self.d_plot = self.v.line(d, X=np.array([self.t]), opts=dict(
            title = "Altitude (meters)",
            xlabel = 'Timestep',
            ylabel = 'Up'
        ))
        ######## ------------ End of Visualization Code ------------------- #########


    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:
            # coordinate conversion
            altitude = -1.0 * self.local_position[2]
            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                #self.landing_transition()
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            # From Backyard Flyer solution
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                # check if we have any waypoints for the drone to fly
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:
            if((self.global_position[2] - self.global_home[2] < 0.1) and abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

    ######## ------------ Visualization Callback Methods ------------------- #########

    def update_ne_plot_callback(self):
        # Note [0] is North and [1] is East, for Plotting purpose, they are switched
        ne = np.array([self.local_position[1], self.local_position[0]]).reshape(1,-1)
        self.v.scatter(ne, win=self.ne_plot, update='append')

    def update_altitude_plot_callback(self):
        # Multiplying by -1.0 just for plotting purposes - to plot up vs. down
        d = -1.0 * np.array([self.local_position[2]])
        # update timestamp
        self.t += 1
        self.v.line(d, X=np.array([self.t]), win=self.d_plot, update='append')

    def calculate_box(self):
        """
        1. Return waypoints to fly a box
        """
        print ("Setting Waypoints")
        #10,0,3,0 = pitch forward
        #10,10,3,0 = roll right
        #0,10,3,0 = pitch backward
        #0,0,3,0 = roll left
        drone_path_waypoints = [[10.0,0.0,3.0,0.0], [10.0,10.0,3.0,0.0], [0.0,10.0,3.0,0.0], [0.0,0.0,3.0,0.0]]
        return drone_path_waypoints

        '''
        for corner in drone_path_waypoints:
            drone.cmd_position(*corner)
            time.sleep(3)
        '''

    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],
        self.global_position[1], self.global_position[2])

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        # The method pop() removes and returns last object or obj from the list.
        target_waypoint = self.all_waypoints.pop(0) # remove current waypoint and get the next waypoint
        self.target_position = target_waypoint
        print('target position', target_waypoint)
        #self.target_position[3] is always 0.0 (since that is what it is set in the drone_path_waypoints)
        # From Drone API documentation
        # cmd_position(north, east, altitude, heading) command the drone to move to a specific
        # (N, E, altitude) defined position (in meters) with a specific heading (in radians)
        north_pos = target_waypoint[0]
        east_pos = target_waypoint[1]
        altitude = target_waypoint[2]
        heading_in_rads = target_waypoint[3]

        self.cmd_position(north_pos, east_pos, altitude, heading_in_rads)
        #self.cmd_position(target_waypoint[0], target_waypoint[1], target_waypoint[2], target_waypoint[3])
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method

        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "TLog.txt") #Telemetry log file
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
