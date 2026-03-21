"""
High-level mission driver for orchestrating drone missions
Manages mission lifecycle, logging, and execution
"""
import json
import time
from .types import MissionState
from .controller import MissionController


class Driver:
    """
    High-level driver for managing drone missions
    Handles mission initialization, execution, and logging
    Designed to run efficiently on Jetson embedded systems
    """
    
    def __init__(self):
        """Initialize the driver"""
        self.missions = {}
        self.current_mission = None
        self.mission_logs = []
        print("[DRIVER] Mission controller driver initialized")
    
    def create_mission(self, mission_id, site_gps, mission_boundary, home_position, 
                       num_laps=3, strategy=None):
        """
        Create a new mission
        
        Args:
            mission_id: Unique identifier for the mission
            site_gps: Target site GPS coordinates
            mission_boundary: Mission area boundary
            home_position: Home/launch location
            num_laps: Number of laps to fly
            strategy: Optional MissionStrategy (defaults to MissionOne)
            
        Returns:
            The created MissionController instance
        """
        mission = MissionController(mission_id, site_gps, mission_boundary, 
                                   home_position, num_laps, strategy)
        self.missions[mission_id] = mission
        print(f"[DRIVER] Created mission {mission_id}")
        return mission
    
    def start_mission(self, mission_id):
        """
        Start execution of a mission
        
        Args:
            mission_id: ID of mission to start
        """
        if mission_id not in self.missions:
            print(f"[DRIVER] ERROR: Mission {mission_id} not found")
            return False
        
        self.current_mission = self.missions[mission_id]
        print(f"[DRIVER] Starting mission {mission_id}")
        
        try:
            self.current_mission.run()
            self.log_mission(mission_id, "COMPLETED")
            return True
        except Exception as e:
            print(f"[DRIVER] ERROR: Mission {mission_id} failed: {str(e)}")
            self.log_mission(mission_id, f"FAILED: {str(e)}")
            return False
    
    def abort_mission(self, mission_id):
        """
        Abort a running mission
        
        Args:
            mission_id: ID of mission to abort
        """
        if mission_id in self.missions:
            mission = self.missions[mission_id]
            mission.state = MissionState.RETURN_HOME
            print(f"[DRIVER] Aborted mission {mission_id}")
            self.log_mission(mission_id, "ABORTED")
    
    def log_mission(self, mission_id, status):
        """Log mission execution"""
        log_entry = {
            "mission_id": mission_id,
            "status": status,
            "timestamp": time.time()
        }
        self.mission_logs.append(log_entry)
        print(f"[DRIVER] Logged mission {mission_id}: {status}")
    
    def get_mission_status(self, mission_id):
        """Get status of a mission"""
        if mission_id in self.missions:
            return self.missions[mission_id].get_mission_status()
        return None
    
    def list_missions(self):
        """List all missions"""
        print("[DRIVER] Registered missions:")
        for mission_id in self.missions.keys():
            print(f"  - Mission {mission_id}")
    
    def export_logs(self, filename):
        """
        Export mission logs to JSON file
        Useful for post-flight analysis on Jetson
        
        Args:
            filename: Output filename for logs
        """
        try:
            with open(filename, 'w') as f:
                json.dump(self.mission_logs, f, indent=2)
            print(f"[DRIVER] Logs exported to {filename}")
        except Exception as e:
            print(f"[DRIVER] ERROR: Failed to export logs: {str(e)}")
