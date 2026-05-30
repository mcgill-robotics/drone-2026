# drone-2026
Welcome to the McGill Drone 2026 repository!

# Competition Documentation
Head to the [wiki](https://github.com/mcgill-robotics/drone-2026/wiki) for setup documentation and general info.

# Who we are
McGill Robotics is a student-led engineering design team dedicated to the development of competition and research robots. Composed of approximately 200 students across three major projects: Autonomous Underwater Vehicle, Mars Rover, and Unmanned Aerial Vehicle. The team's mission is to inspire students through inclusive access to technical resources, mentorship, and hands-on learning. As the youngest initiative, the McGill Robotics Drone Team embodies the organization's commitment to innovation and technical growth. The team is comprised of roughly 40 members collaborating across subteams and is participating in the AEAC competition for the first time with the development of an electric VTOL aircraft. Guided by the motto "Team Before Machine," the team emphasizes collaboration, sustainability, and responsible engineering practices.

# Competition Resources
- **Competition Website:** [Aerial Evolution Annual Student Competition](https://www.aerialevolution.ca/annual-student-competition/)
- **Competition Documentation:** [2026 AEAC CONOPS v1.3](https://www.aerialevolution.ca/wp-content/uploads/2026/03/2026-AEAC-CONOPS-v1.3-2026-03-03.pdf)

# Competition details
## Task 1: Fire Reconnaissance
The objective of this task is to reconnoiter a site and deliver firefighting equipment. 
### General Overview
- Equipment Delivery: Teams must transport and release simulated firefighting equipment (a handheld radio, an oxygen tank, and/or a ladder) to a "distant scene."
- Distance Simulation: To simulate travel to a distant site, UAVs must perform laps of a course between 400m and 1km in length. 
- Target Identification: Bidders must identify the location and color of fire targets at the scene. 
### Rules and Restrictions
- UAV Limit: A maximum of two UAVs are permitted for this task. 
- Payload Handling: Equipment must be manually attached at the flight line and carried in a single run; returning for more items is not allowed. 
- Lap Counting: If multiple UAVs are used, the lowest number of laps completed by any single UAV will be used for the entire team's score. 
- Reporting: Target locations must be submitted in a single .txt file by the end of the flight window. 
- Maintenance: Battery swaps are prohibited during this task. 
## Task 2: Fire Extinguishing
The objective of this task is to extinguish small indoor and outdoor blazes while demonstrating system autonomy. 
### General Overview
- Target Types: Targets are paper circles (5cm to 30cm in diameter) located both outdoors and inside a building. 
- Extinguishing Criteria: A target is "extinguished" when wetted by the UAS across a 2cm wide area. 
- Verification: Operators must declare a target extinguished and provide a photo in real time to judges for confirmation. 
### Rules and Restrictions
- UAV Limit: Only one UAV is permitted for this task. 
- Indoor Access: Indoor targets are reached through a 4m x 4m open doorway. 
- Prohibited Materials: Only water can touch the targets; no part of the UAS may make contact. No items (like beacons) may be left behind in the building. 
- Safety: Compressed gas (like CO2 cartridges) is generally prohibited as it is considered a hazardous payload. 
- Autonomy: Points are awarded for autonomy, but no partial points are given for "partial" autonomous sequences. 
## Common Flight Restrictions
- Flight Boundaries: Teams must stay within "soft" boundaries (warning) and "hard" boundaries (mandatory flight termination). 
- Altitude: Flights are limited to a maximum altitude of 400ft. 
- Flight Crew: Limited to a maximum of five members on the flight line. 
- Pilot Requirements: All pilots must hold an Advanced RPAS Pilot Certificate. 

## Camera Feed
The Intel RealSense depth camera is streamed over WebRTC via MediaMTX. The `api_server.py` starts MediaMTX and the ffmpeg publishers automatically — no manual setup needed at runtime.

### First-time setup: download MediaMTX

The MediaMTX binary is not committed to this repo. Download the arm64 build and place it in the repo root:

```sh
wget https://github.com/bluenviron/mediamtx/releases/download/v1.18.2/mediamtx_v1.18.2_linux_arm64v8.tar.gz
tar -xzf mediamtx_v1.18.2_linux_arm64v8.tar.gz mediamtx
rm mediamtx_v1.18.2_linux_arm64v8.tar.gz
```

The included `mediamtx.yml` configures the `depth` and `rgb` stream paths. `api_server.py` picks it up automatically on startup.
