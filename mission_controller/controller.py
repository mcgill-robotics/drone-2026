"""
Main mission controller implementing FSM state machine
Core logic for managing drone mission execution
"""
import time
from .types import MissionState, Mode, MissionType
from .strategies import MissionOne, MissionTwo
from .stubs import (
    takeoff_drone, land_drone, goto_drone, boustrophedon_search,
    at_position, pad_has_extinguisher, drop_payload, inside_boundary
)


class MissionController:
    """Main mission controller FSM for drone operations"""

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

    def run(self):
        """
        Main mission control loop. Executes state machine until mission complete.
        Routes to Mission One or Mission Two FSM based on mission_type.
        """
        print(f"\n{'='*60}")
        print(f"MISSION {self.mission_number} STARTED")
        print(f"Type: {self.mission_type.name}")
        print(f"Strategy: {self.mission_strategy.get_current_mission()}")
        print(f"{'='*60}\n")
        
        # Route to appropriate state machine
        if self.mission_type == MissionType.MISSION_ONE:
            self._run_mission_one()
        elif self.mission_type == MissionType.MISSION_TWO:
            self._run_mission_two()
        
        print(f"\n{'='*60}")
        print(f"MISSION {self.mission_number} COMPLETE")
        print(f"{'='*60}\n")
    
    def _run_mission_one(self):
        """
        Mission One state machine: LAPS at home → TRANSIT → SEARCH → DROP → RETURN → LAND
        """
        while self.state != MissionState.COMPLETE:
            # Check for mission timeout
            self.check_timeout()
            
            # Update telemetry
            self.update_telemetry()

            # Execute current state
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

            time.sleep(0.1)  # Control loop timing
    
    def _run_mission_two(self):
        """
        Mission Two state machine: ENTER_BUILDING → SEARCH_BUILDING → SPRAY_PADS → EXIT_BUILDING → RETURN → LAND
        """
        while self.state != MissionState.COMPLETE:
            # Check for mission timeout
            self.check_timeout()
            
            # Update telemetry
            self.update_telemetry()

            # Execute current state
            if self.state == MissionState.INIT:
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

            time.sleep(0.1)  # Control loop timing

    def initialize(self):
        """
        Initialize mission - set start time and transition to takeoff
        """
        self.mission_start_time = time.time()
        print(f"[INIT] Mission {self.mission_number} initialized at {self.mission_start_time}")
        print(f"[INIT] Target site: {self.site_gps}")
        print(f"[INIT] Home position: {self.home_position}")
        self.state = MissionState.TAKEOFF

    def takeoff(self):
        """
        Execute takeoff sequence.
        Transitions to appropriate next state based on mission type.
        """
        print("[TAKEOFF] Beginning takeoff sequence")
        self.current_mode = Mode.ASCEND
        takeoff_drone()
        print("[TAKEOFF] Takeoff complete")
        self.current_mode = Mode.AIRBORNE
        
        # Branch based on mission type
        if self.mission_type == MissionType.MISSION_ONE:
            print("[TAKEOFF] Mission One: Beginning laps")
            self.state = MissionState.LAPS
        elif self.mission_type == MissionType.MISSION_TWO:
            print("[TAKEOFF] Mission Two: Navigating to building entry")
            self.state = MissionState.ENTER_BUILDING

    def do_laps(self):
        """
        Execute lap pattern at home location
        """
        print(f"[LAPS] Beginning lap {self.laps_completed + 1}/{self.lap_target}")
        self.mission_strategy.execute()
        
        self.laps_completed += 1
        print(f"[LAPS] Lap {self.laps_completed} completed")

        if self.laps_completed >= self.lap_target:
            print(f"[LAPS] All {self.lap_target} laps completed, transiting to site")
            self.state = MissionState.TRANSIT_TO_SITE
        else:
            print(f"[LAPS] {self.lap_target - self.laps_completed} laps remaining")

    def go_to_site(self):
        """
        Navigate to the target site while respecting mission boundaries
        """
        print(f"[TRANSIT] Navigating to site at {self.site_gps}")
        self.safe_goto(self.site_gps, self.boundary)

        if at_position(self.site_gps):
            print("[TRANSIT] Arrived at site, beginning search")
            self.state = MissionState.SEARCH_SITE

    def search_site(self):
        """
        Execute search pattern to locate landing pad
        """
        print("[SEARCH] Beginning site search with boustrophedon pattern")
        pad = boustrophedon_search()

        if pad is not None:
            print(f"[SEARCH] Pad detected at {pad}")
            self.detected_pad = pad
            self.state = MissionState.DROP_PAYLOAD
        else:
            print("[SEARCH] No pad detected, returning home")
            self.state = MissionState.RETURN_HOME

    def handle_drop(self):
        """
        Execute payload drop if conditions are met
        """
        print("[DROP] Handling payload drop")
        
        if not self.payload_available:
            print("[DROP] No payload available, skipping drop")
            self.state = MissionState.RETURN_HOME
            return

        # Check if pad already has extinguisher
        if not pad_has_extinguisher(self.detected_pad):
            print(f"[DROP] Dropping payload at {self.detected_pad}")
            drop_payload(self.detected_pad)
            self.payload_available = False
            print("[DROP] Payload dropped successfully")
        else:
            print("[DROP] Pad already has extinguisher, skipping drop")

        self.state = MissionState.RETURN_HOME

    def check_timeout(self):
        """
        Monitor mission elapsed time and abort to home if timeout exceeded
        """
        if self.mission_start_time is None:
            return

        elapsed_time = time.time() - self.mission_start_time
        
        if elapsed_time > self.mission_duration_limit:
            print(f"[TIMEOUT] Mission exceeded {self.mission_duration_limit}s limit. Aborting to home.")
            self.state = MissionState.RETURN_HOME

    def return_home(self):
        """
        Navigate back to home location while respecting mission boundaries
        """
        print(f"[RETURN] Returning to home at {self.home_position}")
        self.safe_goto(self.home_position, self.boundary)

        if at_position(self.home_position):
            print("[RETURN] Arrived at home, preparing to land")
            self.state = MissionState.LAND

    def land(self):
        """
        Execute landing sequence
        """
        print("[LAND] Beginning landing sequence")
        self.current_mode = Mode.LAND
        land_drone()
        print("[LAND] Landing complete, mission finished")
        self.current_mode = Mode.HOVER
        self.state = MissionState.COMPLETE

    def enter_building(self):
        """
        Navigate to building entry point and enter the building (Mission Two)
        """
        print(f"[ENTER] Navigating to building entry at {self.building_entry_point}")
        self.safe_goto(self.building_entry_point, self.boundary)

        if at_position(self.building_entry_point):
            print("[ENTER] Arrived at building entry, entering building")
            self.building_interior = True
            self.state = MissionState.SEARCH_BUILDING

    def search_building(self):
        """
        Execute search pattern inside building to locate pads (Mission Two)
        """
        print("[SEARCH_BUILDING] Beginning interior search for pads")
        pads = boustrophedon_search()  # Reuse search logic for interior

        if pads:
            print(f"[SEARCH_BUILDING] Found {len(pads) if isinstance(pads, list) else 1} pad(s)")
            self.state = MissionState.SPRAY_PADS
        else:
            print("[SEARCH_BUILDING] No pads detected, exiting building")
            self.state = MissionState.EXIT_BUILDING

    def spray_pads(self):
        """
        Execute spray/extinguish on located pads (Mission Two)
        """
        print("[SPRAY] Handling payload spray on detected pads")
        
        if isinstance(self.mission_strategy, MissionTwo):
            # Use Mission Two's spray capability
            if self.mission_strategy.current_water_level <= 0:
                print("[SPRAY] Water tank empty, exiting building")
                self.state = MissionState.EXIT_BUILDING
                return
            
            # Spray at detected location
            if self.detected_pad:
                self.mission_strategy.spray_water(self.detected_pad)
                print(f"[SPRAY] Applied extinguisher at {self.detected_pad}")
            
            self.state = MissionState.EXIT_BUILDING
        else:
            print("[SPRAY] Mission Two strategy not configured properly")
            self.state = MissionState.EXIT_BUILDING

    def exit_building(self):
        """
        Navigate to building exit point and leave the building (Mission Two)
        """
        print(f"[EXIT] Navigating to building exit at {self.building_exit_point}")
        self.safe_goto(self.building_exit_point, self.boundary)

        if at_position(self.building_exit_point):
            print("[EXIT] Exited building, returning to home")
            self.building_interior = False
            self.state = MissionState.RETURN_HOME

    def safe_goto(self, target, boundary):
        """
        Navigate to target while respecting mission boundaries
        
        Args:
            target: GPS coordinates to navigate to
            boundary: Mission boundary constraints
            
        Raises:
            Exception: If target is outside mission boundaries
        """
        if not inside_boundary(target, boundary):
            raise Exception(f"Target {target} is outside mission boundary!")
        
        print(f"[GOTO] Navigating to {target}")
        goto_drone(target)
    
    def update_telemetry(self):
        """
        Update drone telemetry from ardupilot
        This should be called regularly to keep state synchronized
        """
        # TODO: Implement real telemetry reading from ardupilot
        # self.current_location = read_current_position()
        # self.battery_level = read_battery_level()
        # self.altitude = read_altitude()
        # self.current_mode = read_flight_mode()
        pass
    
    def add_objective(self, objective):
        """Add an objective to this mission"""
        self.objectives.append(objective)
    
    def set_mission_strategy(self, strategy):
        """Switch to a different mission strategy"""
        self.mission_strategy = strategy
        print(f"[CONTROLLER] Mission strategy switched to {strategy.get_current_mission()}")
    
    def get_mission_status(self):
        """Get current mission status"""
        return {
            "mission_number": self.mission_number,
            "state": self.state.name,
            "laps_completed": self.laps_completed,
            "battery": self.battery_level,
            "location": str(self.current_location),
            "strategy": self.mission_strategy.get_current_mission()
        }
