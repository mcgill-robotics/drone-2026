"""
High-level mission driver for orchestrating drone missions
"""
import json
import time
from types import MissionState
from controller import MissionController


class Driver:
    """
    this class is a mission manager that handles creating, executing
    and logging drone missions. You need to instantiate a Driver object
    to then run the missions. this is the highest level of abstraction
    in the codebase. You can run stuff like driver.creat_mission() and driver.start_mission()
    """
    
    def __init__(self):
        """Initialize the driver. A driver can manage many missions"""
        self.missions = {}
        self.current_mission = None
        self.mission_logs = []
    
    def create_mission(self, mission_id, site_gps, mission_boundary, home_position, 
                       num_laps=3, strategy=None):
        """Create a new mission"""
        mission = MissionController(mission_id, site_gps, mission_boundary, 
                                   home_position, num_laps, strategy)
        self.missions[mission_id] = mission
        return mission
    
    def start_mission(self, mission_id):
        """Start execution of a mission"""
        if mission_id not in self.missions:
            return False
        # selects the mission to run from mission_id
        self.current_mission = self.missions[mission_id]
        
        try:
            #runs MissionController.run() which executes the mission FSM until completion or failure
            self.current_mission.run()
            self.log_mission(mission_id, "COMPLETED")
            return True
        except Exception as e:
            self.log_mission(mission_id, f"FAILED: {str(e)}")
            return False
    
    def abort_mission(self, mission_id):
        """Abort a running mission"""
        if mission_id in self.missions:
            mission = self.missions[mission_id]
            mission.state = MissionState.RETURN_HOME
            self.log_mission(mission_id, "ABORTED")
    
    def log_mission(self, mission_id, status):
        """Log mission execution"""
        log_entry = {
            "mission_id": mission_id,
            "status": status,
            "timestamp": time.time()
        }
        self.mission_logs.append(log_entry)
    
    def get_mission_status(self, mission_id):
        """Get status of a mission"""
        if mission_id in self.missions:
            return self.missions[mission_id].get_mission_status()
        return None
    
    def list_missions(self):
        """List all missions"""
        return list(self.missions.keys())
    
    def export_logs(self, filename):
        """Export mission logs to JSON file"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.mission_logs, f, indent=2)
            return True
        except Exception:
            return False
