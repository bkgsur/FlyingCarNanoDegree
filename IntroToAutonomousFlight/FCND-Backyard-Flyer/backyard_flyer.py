# -*- coding: utf-8 -*-
"""
Solution to the Backyard Flyer Project.
"""

import time
from enum import Enum
import datetime as dt
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import sys
import matplotlib.pyplot as plt
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

    def __init__(self, connection,x,z):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.position_list_x= []
        self.position_list_y= []
        self.position_list_z= []
        self.in_mission = True
        self.check_state = {}
        self.height = int(z)
        self.width = int(x)
        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)    
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)   
        self.register_callback(MsgID.STATE, self.state_callback)

        self.register_state_transition(States.TAKEOFF,self.waypoint_transition)
        self.register_state_transition(States.WAYPOINT,self.landing_transition)
        self.register_state_transition(States.LANDING,self.disarming_transition)
        self.register_state_transition(States.MANUAL,self.arming_transition)
        self.register_state_transition(States.ARMING,self.takeoff_transition)
        self.register_state_transition(States.DISARMING,self.manual_transition)

    def register_state_transition(self, state, transition):
        if state not in self.check_state:
            self.check_state[state] = []
        self.check_state[state].append(transition)

    def handle_state_transition(self, state):
        if state not in self.check_state:
            print("sorry, I don't understand", state)
        else:
            callbacks = self.check_state[state]
            for callback in callbacks:
                callback() 

    def local_position_callback(self):
        self.position_list_x.append(self.local_position[0])
        self.position_list_y.append(self.local_position[1])
        self.position_list_z.append(-1 * self.local_position[2])
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.all_waypoints = [[self.width, 0.0, self.height], [self.width, self.width, self.height], [0.0, self.width, self.height], [0.0, 0.0, self.height]]
                self.handle_state_transition(self.flight_state)                
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 0.05: # eucledian distance
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()                     
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 0.05:
                         self.handle_state_transition(self.flight_state)                              

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.05:
                if abs(self.local_position[2]) < 0.05:
                    self.handle_state_transition(self.flight_state)   

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.handle_state_transition(self.flight_state)
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.handle_state_transition(self.flight_state)
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.handle_state_transition(self.flight_state)    

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0], self.global_position[1],
                               self.global_position[2])  # set the current location to be the home position

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")         
        self.target_position[2] = self.height
        self.takeoff(self.height)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        self.target_position = self.all_waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        super().start()
        self.stop_log()
        self.draw_graph()

    def draw_graph(self):                
        ax = plt.axes(projection='3d')
        ax.set_title('Drone position during flight')
        ax.set_zlabel('Elevation ') 
        ax.plot3D(self.position_list_x, self.position_list_y, self.position_list_z, 'red')
        plt.draw()
        plt.show()
        
if __name__ == "__main__":
    ar = sys.argv
    #default deimensions of flight
    x=20
    z=3
    if(len(ar)==3):
        x= ar[1]
        z= ar[2]

    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    drone = BackyardFlyer(conn,x,z)
    time.sleep(2)
    drone.start()
   
