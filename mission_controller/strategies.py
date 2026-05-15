"""
Mission strategy classes implementing different mission behaviors
Uses the Strategy pattern for flexible mission planning
"""
from abc import ABC, abstractmethod
from stubs import run_lap_algorithm, goto_drone


class MissionStrategy(ABC):
    """Abstract base class for different mission strategies"""
    
    def __init__(self, mission_boundary):
        """
        Initialize mission strategy
        
        Args:
            mission_boundary: Boundary constraints for mission
        """
        self.mission_boundary = mission_boundary
        self.visited_objectives = []
        self.battery_level = 100.0
        self.current_objective = None
        self.pathfinding = None
    
    @abstractmethod
    def execute(self):
        """
        Execute the mission strategy
        Subclasses implement specific mission logic
        """
        pass
    
    @abstractmethod
    def nearest_potential_field(self, point):
        """Calculate nearest potential field point"""
        pass
    
    def update_battery(self, new_level):
        """Update current battery percentage"""
        self.battery_level = new_level
        print(f"[MISSION] Battery level: {self.battery_level}%")
    
    def add_visited_objective(self, objective):
        """Record a visited objective"""
        self.visited_objectives.append(objective)
    
    def get_current_mission(self):
        """Get current mission strategy"""
        return self.__class__.__name__
    
    def switch_pathfinding_strategy(self, strategy):
        """Switch to a different pathfinding strategy"""
        self.pathfinding = strategy
        print(f"[MISSION] Switched pathfinding to {strategy.__class__.__name__}")


class MissionOne(MissionStrategy):
    """Mission One - Lap-based mission with item tracking"""
    
    def __init__(self, mission_boundary, lap_waypoints=None, item_count=0):
        """
        Initialize Mission One
        
        Args:
            mission_boundary: Mission area boundaries
            lap_waypoints: List of Point objects for lap pattern
            item_count: Number of items to track
        """
        super().__init__(mission_boundary)
        self.lap_waypoints = lap_waypoints if lap_waypoints else []
        self.item_count = item_count
        self.items_found = 0
        self.num_laps = 0
    
    def execute(self):
        """
        Execute Mission One - fly laps then go to site
        """
        print("[MISSION ONE] Executing lap-based search mission")
        run_lap_algorithm()
        self.num_laps += 1
    
    def nearest_potential_field(self, point):
        """Find nearest potential field waypoint"""
        print(f"[MISSION ONE] Finding nearest potential field to {point}")
        # TODO: Implement nearest waypoint search
        return self.lap_waypoints[0] if self.lap_waypoints else point
    
    def take_picture(self):
        """Take picture during mission"""
        print("[MISSION ONE] Taking picture")
        # TODO: Integrate with vision system
        pass
    
    def navigate_to_point(self, point):
        """Navigate to specific point"""
        print(f"[MISSION ONE] Navigating to {point}")
        goto_drone(point)
    
    def mark_visited(self, point):
        """Mark a location as visited"""
        print(f"[MISSION ONE] Marked {point} as visited")
        self.visited_objectives.append(point)
    
    def increment_lap(self):
        """Increment lap counter"""
        self.num_laps += 1
        print(f"[MISSION ONE] Lap {self.num_laps} completed")


class MissionTwo(MissionStrategy):
    """Mission Two - Water tank based mission with capacity management"""
    
    def __init__(self, mission_boundary, water_tank_capacity=100.0):
        """
        Initialize Mission Two
        
        Args:
            mission_boundary: Mission area boundaries
            water_tank_capacity: Maximum water tank capacity in liters
        """
        super().__init__(mission_boundary)
        self.water_tank_capacity = water_tank_capacity
        self.current_water_level = water_tank_capacity
        self.objectives = []
    
    def execute(self):
        """
        Execute Mission Two - spray/extinguish based mission
        """
        print("[MISSION TWO] Executing water/extinguisher mission")
        for objective in self.objectives:
            if self.current_water_level <= 0:
                print("[MISSION TWO] No water remaining, aborting")
                break
            objective.execute()
            self.decrement_water_tank_capacity()
    
    def nearest_potential_field(self, point):
        """Find nearest potential field waypoint"""
        print(f"[MISSION TWO] Finding nearest potential field to {point}")
        # TODO: Implement nearest objective search
        return self.objectives[0] if self.objectives else point
    
    def add_objective(self, objective):
        """Add an objective to this mission"""
        self.objectives.append(objective)
        print(f"[MISSION TWO] Added objective at {objective.location}")
    
    def decrement_water_tank_capacity(self, amount=10.0):
        """Decrement water level after usage"""
        self.current_water_level = max(0, self.current_water_level - amount)
        print(f"[MISSION TWO] Water level: {self.current_water_level}/{self.water_tank_capacity}L")
    
    def increment_water_tank_capacity(self, amount=100.0):
        """Refill water tank"""
        self.current_water_level = min(self.water_tank_capacity, 
                                       self.current_water_level + amount)
        print(f"[MISSION TWO] Water refilled to {self.current_water_level}L")
    
    def spray_water(self, target):
        """Spray water at target location"""
        if self.current_water_level > 0:
            print(f"[MISSION TWO] Spraying water at {target}")
            # TODO: Integrate with sprayer mechanism
            self.decrement_water_tank_capacity(5.0)
        else:
            print("[MISSION TWO] No water available to spray")
