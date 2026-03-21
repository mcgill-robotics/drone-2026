"""
Objective classes for mission planning
Objectives represent specific tasks to be accomplished during a mission
"""
from abc import ABC, abstractmethod
from .stubs import extinguish_fire, take_survey_photos, release_payload


class Objective(ABC):
    """Abstract base class for mission objectives"""
    
    def __init__(self, location):
        """
        Initialize an objective
        
        Args:
            location: Point object indicating objective location
        """
        self.location = location
        self.complete = False
    
    @abstractmethod
    def execute(self):
        """
        Execute the objective action
        Subclasses must implement this
        """
        pass
    
    def is_complete(self):
        """Check if objective is completed"""
        return self.complete
    
    def set_complete(self):
        """Mark objective as completed"""
        self.complete = True
    
    def to_dict(self):
        """Serialize objective to dictionary"""
        return {
            "type": self.__class__.__name__,
            "location": self.location.to_dict(),
            "complete": self.complete
        }


class ExtinguishObjective(Objective):
    """Objective to extinguish a fire at a specific location"""
    
    def __init__(self, location):
        """
        Initialize fire extinguishing objective
        
        Args:
            location: Point object of fire location
        """
        super().__init__(location)
        self.fire_detected = False
    
    def execute(self):
        """
        Execute fire extinguishing procedure
        This is called when drone reaches the fire location
        """
        print(f"[OBJECTIVE] Executing fire extinguish at {self.location}")
        extinguish_fire(self.location)
        self.set_complete()
    
    def detect_fire(self):
        """Fire detection logic - stub"""
        print(f"[OBJECTIVE] Detecting fire at {self.location}")
        # TODO: Implement fire detection using vision/thermal sensors
        self.fire_detected = True
        return self.fire_detected


class SurveyObjective(Objective):
    """Objective to survey/photograph a specific area"""
    
    def __init__(self, location):
        """
        Initialize survey objective
        
        Args:
            location: Point object of area to survey
        """
        super().__init__(location)
        self.photos_taken = 0
    
    def execute(self):
        """
        Execute survey/photography at location
        """
        print(f"[OBJECTIVE] Executing survey at {self.location}")
        self.photos_taken = take_survey_photos(self.location)
        print(f"[OBJECTIVE] Survey complete - {self.photos_taken} photos taken")
        self.set_complete()
    
    def record_item(self):
        """Record item detected during survey"""
        # TODO: Implement item logging and tracking
        pass


class PayloadDeliveryObjective(Objective):
    """Objective to deliver payload at a specific location"""
    
    def __init__(self, location):
        """
        Initialize payload delivery objective
        
        Args:
            location: Point object where payload should be delivered
        """
        super().__init__(location)
        self.payload_type = None
    
    def execute(self):
        """
        Execute payload delivery at location
        """
        print(f"[OBJECTIVE] Executing payload delivery at {self.location}")
        release_payload(self.location)
        self.set_complete()
    
    def set_payload_type(self, payload_type):
        """Set the type of payload being delivered"""
        self.payload_type = payload_type
