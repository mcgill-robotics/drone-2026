"""
Main mission controller implementing FSM state machine
Core logic for managing drone mission execution
"""
import time
from .types import MissionState, Mode, MissionType
from .strategies import MissionOne, MissionTwo
from .px4_interface import get_px4
from .stubs import (
    goto_drone, boustrophedon_search,
    at_position, pad_has_extinguisher, drop_payload, inside_boundary
)


class MissionController:
    """Main mission controller FSM for drone operations. This is the second level
    of abstraction in the codebase, with Driver being the highest level."""

    def __init__(self, mission_number, site_gps, mission_boundary, home_position, 
                 num_laps=3, mission_strategy=None, building_entry_point=None, 
                 building_exit_point=None):
        """
        Initialize the mission controller
        
        Args:
            mission_number: Identifier for the mission
            site_gps: GPS coordinates of the target site (outdoor for M1, building for M2)
            mission_boundary: Boundary constraints for the mission area
            home_position: GPS coordinates of home/launch location
            num_laps: Number of laps to complete before transiting to site (Mission One only)
            mission_strategy: MissionStrategy object (defaults to MissionOne)
            building_entry_point: GPS coordinates for entering building (Mission Two only)
            building_exit_point: GPS coordinates for exiting building (Mission Two only)
        """
        # FSM state
        self.state = MissionState.INIT
        
        # Mission type detection from strategy
        if mission_strategy is None:
            #MissionOne comes from strategies.py
            self.mission_strategy = MissionOne(mission_boundary)
            self.mission_type = MissionType.MISSION_ONE
        else:
            self.mission_strategy = mission_strategy
            if isinstance(mission_strategy, MissionTwo):
                self.mission_type = MissionType.MISSION_TWO
            else:
                self.mission_type = MissionType.MISSION_ONE
        
        # Mission parameters
        self.mission_number = mission_number
        self.site_gps = site_gps
        self.boundary = mission_boundary
        self.home_position = home_position
        
        # Building-specific parameters (Mission Two only)
        self.building_entry_point = building_entry_point
        self.building_exit_point = building_exit_point
        self.building_interior = False  # Track if drone is inside building
        
        # Lap tracking (Mission One only)
        self.lap_target = num_laps
        self.laps_completed = 0
        
        # Payload management (Mission One)
        self.payload_available = True
        self.detected_pad = None
        
        # Mission timing
        self.mission_start_time = None
        self.mission_duration_limit = 30 * 60  # 30 minutes in seconds
        
        # Flight telemetry
        self.current_mode = Mode.AIRBORNE
        self.battery_level = 100.0
        self.current_location = home_position
        self.altitude = 0
        
        # Objectives
        self.objectives = []


    #Runs the mission depending on the type. they are defined below the run file
    def run(self):
        """Main mission control loop"""
        print(f"MISSION {self.mission_number} STARTED - Type: {self.mission_type.name}")
        
        if self.mission_type == MissionType.MISSION_ONE:
            self._run_mission_one()
        elif self.mission_type == MissionType.MISSION_TWO:
            self._run_mission_two()
        
        print(f"MISSION {self.mission_number} COMPLETE")
    
    def _run_mission_one(self):
        """Mission One state machine"""
        while self.state != MissionState.COMPLETE:
            self.check_timeout()
            self.update_telemetry()

            if self.state == MissionState.INIT:
                self.initialize()
            elif self.state == MissionState.TAKEOFF:
                self.takeoff()
            elif self.state == MissionState.LAPS:
                self.do_laps()
            elif self.state == MissionState.TRANSIT_TO_SITE:
                self.go_to_site()
            elif self.state == MissionState.SEARCH_SITE:
                self.search_site()
            elif self.state == MissionState.DROP_PAYLOAD:
                self.handle_drop()
            elif self.state == MissionState.RETURN_HOME:
                self.return_home()
            elif self.state == MissionState.LAND:
                self.land()

            time.sleep(0.1)
    
    def _run_mission_two(self):
        """Mission Two state machine"""
        while self.state != MissionState.COMPLETE:
            self.check_timeout()
            self.update_telemetry()

            if self.state == MissionState.INIT:
                # just sets start time and changes state to takeoff
                self.initialize()
            elif self.state == MissionState.TAKEOFF:
                self.takeoff()
            elif self.state == MissionState.ENTER_BUILDING:
                self.enter_building()
            elif self.state == MissionState.SEARCH_BUILDING:
                self.search_building()
            elif self.state == MissionState.SPRAY_PADS:
                self.spray_pads()
            elif self.state == MissionState.EXIT_BUILDING:
                self.exit_building()
            elif self.state == MissionState.RETURN_HOME:
                self.return_home()
            elif self.state == MissionState.LAND:
                self.land()

            time.sleep(0.1)

    def initialize(self):
        """Initialize mission"""
        self.mission_start_time = time.time()
        self.state = MissionState.TAKEOFF

    def takeoff(self):
        """Execute takeoff"""
        self.current_mode = Mode.ASCEND
        autopilot = get_px4()
        if autopilot:
            #50 is the altitude
            autopilot.takeoff(50, timeout=60)
        self.current_mode = Mode.AIRBORNE
        
        if self.mission_type == MissionType.MISSION_ONE:
            self.state = MissionState.LAPS
        elif self.mission_type == MissionType.MISSION_TWO:
            self.state = MissionState.ENTER_BUILDING

    def do_laps(self):
        """Execute lap pattern"""
        self.mission_strategy.execute()
        self.laps_completed += 1

        if self.laps_completed >= self.lap_target:
            self.state = MissionState.TRANSIT_TO_SITE
        else:
            print(f"Lap {self.laps_completed}/{self.lap_target} complete")

    def go_to_site(self):
        """Navigate to target site"""
        self.safe_goto(self.site_gps, self.boundary)
        if at_position(self.site_gps):
            self.state = MissionState.SEARCH_SITE

    def search_site(self):
        """Execute search pattern"""
        pad = boustrophedon_search()
        if pad:
            self.detected_pad = pad
            self.state = MissionState.DROP_PAYLOAD
        else:
            self.state = MissionState.RETURN_HOME

    def handle_drop(self):
        """Execute payload drop"""
        if self.payload_available and not pad_has_extinguisher(self.detected_pad):
            drop_payload(self.detected_pad)
            self.payload_available = False
        self.state = MissionState.RETURN_HOME

    def check_timeout(self):
        """Check mission timeout"""
        if self.mission_start_time and time.time() - self.mission_start_time > self.mission_duration_limit:
            self.state = MissionState.RETURN_HOME

    def return_home(self):
        """Navigate back to home"""
        self.safe_goto(self.home_position, self.boundary)
        if at_position(self.home_position):
            self.state = MissionState.LAND

    def land(self):
        """Execute landing"""
        self.current_mode = Mode.LAND
        autopilot = get_px4()
        if autopilot:
            autopilot.land(timeout=60)
        self.current_mode = Mode.HOVER
        self.state = MissionState.COMPLETE

    def enter_building(self):
        """Navigate to building entry"""
        self.safe_goto(self.building_entry_point, self.boundary)
        if at_position(self.building_entry_point):
            self.building_interior = True
            self.state = MissionState.SEARCH_BUILDING

    def search_building(self):
        """Search building interior"""
        pads = boustrophedon_search()
        if pads:
            self.state = MissionState.SPRAY_PADS
        else:
            self.state = MissionState.EXIT_BUILDING

    def spray_pads(self):
        """Spray detected pads"""
        if isinstance(self.mission_strategy, MissionTwo):
            if self.mission_strategy.current_water_level <= 0:
                self.state = MissionState.EXIT_BUILDING
                return
            if self.detected_pad:
                self.mission_strategy.spray_water(self.detected_pad)
        self.state = MissionState.EXIT_BUILDING

    def exit_building(self):
        """Navigate to building exit"""
        self.safe_goto(self.building_exit_point, self.boundary)
        if at_position(self.building_exit_point):
            self.building_interior = False
            self.state = MissionState.RETURN_HOME

    def safe_goto(self, target, boundary):
        """Navigate to target while respecting boundaries"""
        if not inside_boundary(target, boundary):
            raise Exception(f"Target {target} outside mission boundary!")
        goto_drone(target)
    
    def update_telemetry(self):
        """Update telemetry from autopilot"""
        pass
    
    def add_objective(self, objective):
        """Add objective"""
        self.objectives.append(objective)
    
    def set_mission_strategy(self, strategy):
        """Switch mission strategy"""
        self.mission_strategy = strategy
    
    def get_mission_status(self):
        """Get mission status"""
        return {
            "mission_number": self.mission_number,
            "state": self.state.name,
            "laps_completed": self.laps_completed,
            "battery": self.battery_level,
            "location": str(self.current_location),
            "strategy": self.mission_strategy.get_current_mission()
        }
