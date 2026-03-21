# Mission Controller Package

A comprehensive mission planning and execution system for autonomous drones running on Jetson embedded systems.

## Architecture

The mission controller is organized into modular components matching the UML class diagram:

```
mission_controller/
  __init__.py              # Package exports and API
  types.py                 # Core data types (MissionState, Mode, Point)
  objectives.py            # Mission objectives (ExtinguishObjective, SurveyObjective, etc.)
  pathfinding.py           # Pathfinding strategies (PathPrinting, PotentialFieldPathfinding)
  strategies.py            # Mission strategies (MissionOne, MissionTwo)
  controller.py            # Main MissionController FSM
  driver.py                # High-level mission driver
  stubs.py                 # Placeholder functions for team implementation
  mission_controller.py    # Main entry point with example usage
  README.md                # This file
```

## Components

### Types (`types.py`)
- **MissionState**: Enumeration of FSM states (INIT, TAKEOFF, LAPS, TRANSIT_TO_SITE, SEARCH_SITE, DROP_PAYLOAD, RETURN_HOME, LAND, COMPLETE)
- **Mode**: Drone flight modes (HOVER, LAND, ASCEND, RETURN, AIRBORNE)
- **Point**: GPS/waypoint coordinates with distance calculations and JSON serialization

### Objectives (`objectives.py`)
Abstract mission objectives that can be executed:
- **Objective** (Abstract base)
- **ExtinguishObjective**: Fire extinguishing missions with detection
- **SurveyObjective**: Survey/photography missions
- **PayloadDeliveryObjective**: Payload drop missions

### Pathfinding (`pathfinding.py`)
Navigation and path planning strategies:
- **PathfindingStrategy** (Abstract base)
- **PathPrinting**: Boustrophedon pattern for surveying rectangular areas
- **PotentialFieldPathfinding**: Obstacle avoidance using attractive/repulsive forces

### Strategies (`strategies.py`)
Mission behavior patterns (Strategy pattern):
- **MissionStrategy** (Abstract base)
- **MissionOne**: Lap-based search mission with item tracking
- **MissionTwo**: Water tank management for extinguishing missions

### Controller (`controller.py`)
**MissionController**: Main FSM implementing the mission execution loop
- State machine with 9 states
- Pluggable mission strategy
- Telemetry tracking (battery, altitude, location, mode)
- Timeout management
- Boundary checking

### Driver (`driver.py`)
**Driver**: High-level mission orchestrator
- Mission creation and lifecycle management
- Execution with error handling
- Mission logging and JSON export
- Designed for Jetson efficiency

### Stubs (`stubs.py`)
Placeholder functions for team implementation:
- Flight control: `takeoff_drone()`, `land_drone()`, `goto_drone()`
- Search/detection: `boustrophedon_search()`, `at_position()`, `pad_has_extinguisher()`
- Pathfinding: `generate_print_pattern()`, `generate_potential_field_path()`
- Navigation: `inside_boundary()`
- Actions: `drop_payload()`, `extinguish_fire()`, `take_survey_photos()`, `release_payload()`
- Algorithm: `run_lap_algorithm()`

## Usage

### Basic Example

```python
from mission_controller import (
    Driver, MissionOne, MissionTwo, Point,
    ExtinguishObjective
)

# Create driver
driver = Driver()

# Define mission parameters
home = Point(0, 0, 0)
site = Point(100, 100, 50)
boundary = ((-50, 150), (-50, 150))

# Create lap-based mission
mission = driver.create_mission(
    mission_id=1,
    site_gps=site,
    mission_boundary=boundary,
    home_position=home,
    num_laps=3
)

# Add objectives
mission.add_objective(ExtinguishObjective(Point(110, 110, 50)))

# Execute
driver.start_mission(1)

# Export logs
driver.export_logs("mission_logs.json")
```

### Advanced Example with Custom Strategy

```python
from mission_controller import Driver, MissionTwo, ExtinguishObjective, Point

# Create water-based mission strategy
strategy = MissionTwo(boundary, water_tank_capacity=50.0)
strategy.add_objective(ExtinguishObjective(Point(110, 110, 50)))
strategy.add_objective(ExtinguishObjective(Point(120, 120, 50)))

# Create mission with custom strategy
mission = driver.create_mission(
    mission_id=2,
    site_gps=site,
    mission_boundary=boundary,
    home_position=home,
    num_laps=2,
    strategy=strategy
)

driver.start_mission(2)
```

## Mission Flow

```
INIT → TAKEOFF → LAPS → TRANSIT_TO_SITE → SEARCH_SITE → DROP_PAYLOAD → RETURN_HOME → LAND → COMPLETE
                                                           ↓ (no pad found)
                                                        RETURN_HOME
```

Any state can transition to RETURN_HOME on timeout or emergency.

## Implementation Guide

Team members should implement stub functions in the following order:

1. **Navigation** (`stubs.py`):
   - `takeoff_drone()` - Use ardupilot commands
   - `land_drone()` - Use ardupilot commands
   - `goto_drone(target)` - Navigate to GPS coordinates
   - `inside_boundary(target, boundary)` - Boundary checking

2. **Sensing & Detection**:
   - `at_position(target, tolerance)` - Read drone telemetry
   - `boustrophedon_search()` - Vision-based pad detection
   - `pad_has_extinguisher(location)` - Vision analysis

3. **Algorithms & Pathfinding**:
   - `run_lap_algorithm()` - Implement lap pattern
   - `generate_print_pattern(start, goal)` - Boustrophedon waypoints
   - `generate_potential_field_path(start, goal, obstacles)` - Path planning

4. **Actions**:
   - `drop_payload(target)` - Release mechanism
   - `extinguish_fire(location)` - Sprayer system
   - `take_survey_photos(location)` - Camera integration
   - `release_payload(location)` - Generic payload release

## Jetson Optimization

The system is designed for Jetson efficiency:
- Minimal dependencies (enum, json, time, abc)
- Efficient logging to JSON format
- Modular design allows offloading heavy computation
- No high-bandwidth dependencies
- Clean separation of concerns for parallel development

## Running Tests

```bash
# Run example
python3 mission_controller.py

# Run syntax check
python3 -m py_compile mission_controller/*.py

# Test imports
python3 -c "from mission_controller import *; print('OK')"
```

## Logging

Mission logs are exported as JSON for post-flight analysis:

```json
[
  {
    "mission_id": 1,
    "status": "COMPLETED",
    "timestamp": 1234567890.123
  }
]
```

## TODO

See `[STUB]` and `[TODO]` comments throughout code for implementation points.

## Version

1.0.0
