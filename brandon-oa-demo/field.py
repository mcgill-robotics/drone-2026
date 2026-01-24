# field.py
import math

class Field:
    @staticmethod
    def calculate_attractive_force(position, goal, strength=1.0, max_distance=None):
        """Calculate attractive force towards a goal"""
        dx = goal[0] - position[0]
        dy = goal[1] - position[1]
        distance = math.hypot(dx, dy)
        
        if distance == 0:
            return (0, 0)
        
        # Normalize direction
        direction_x = dx / distance
        direction_y = dy / distance
        
        # Linear attraction (could also use quadratic: strength * distance)
        force_magnitude = strength
        
        # Optional: cap the force at max_distance
        if max_distance and distance > max_distance:
            force_magnitude *= (max_distance / distance)
        
        return (direction_x * force_magnitude, direction_y * force_magnitude)
    
    @staticmethod
    def calculate_repulsive_force(position, obstacle_center, obstacle_radius, 
                                   strength=100.0, influence_distance=150.0):
        """Calculate repulsive force from an obstacle"""
        dx = position[0] - obstacle_center[0]
        dy = position[1] - obstacle_center[1]
        distance = math.hypot(dx, dy)
        
        # Distance from edge of obstacle
        edge_distance = distance - obstacle_radius
        
        if edge_distance <= 0:
            # Inside obstacle - strong repulsion
            if distance == 0:
                return (0, 0)
            direction_x = dx / distance
            direction_y = dy / distance
            return (direction_x * strength * 10, direction_y * strength * 10)
        
        if edge_distance > influence_distance:
            # Too far away - no influence
            return (0, 0)
        
        # Normalize direction (away from obstacle)
        direction_x = dx / distance
        direction_y = dy / distance
        
        # Inverse square law for repulsion (1/d^2)
        force_magnitude = strength / (edge_distance ** 2)
        
        return (direction_x * force_magnitude, direction_y * force_magnitude)