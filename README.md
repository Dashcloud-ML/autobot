# Autobot
# Robotic Simulation with MATLAB

This repository contains a MATLAB script for setting up and simulating a robotic scenario in a 3D environment. The simulation involves a robot navigating through a grid with obstacles and walls. The script demonstrates environment setup, path planning, and visualization of the robot's movement.

## Script Overview

### Initialization

```matlab
scenario = robotScenario(UpdateRate=5);
Adding the Floor
matlab
Copy code
floorColor = [0.5882 0.2941 0];
addMesh(scenario, "Plane", Position=[5 4 0], Size=[10 10], Color=floorColor);
Adds a brownish plane mesh representing the floor.
Adding Outer Walls
matlab
Copy code
wallHeight = 2;
wallWidth = 0.25;
wallLength = 10;
wallColor = [1 1 0.8157];

% Adds four outer walls.
Adds four outer walls with specified dimensions and color.
Adding Inner Walls
matlab
Copy code
% Adds four inner walls with specific dimensions and positions.
Adds inner walls to create obstacles and pathways within the environment.
Displaying the 3D Environment
matlab
Copy code
show3D(scenario);
lightangle(-45,30);
view(30,60);
Displays the 3D environment with specified lighting and view angles.
Creating the Binary Occupancy Map
matlab
Copy code
map = binaryOccupancyMap(scenario, GridOriginInLocal=[-2 -2], MapSize=[15 15], MapHeightLimits=[0 3]);
inflate(map,0.3);
show(map);
Creates and inflates a binary occupancy map representing obstacles in the environment.
Path Planning
matlab
Copy code
startPosition = [1 2];
goalPosition = [8 8];

rng(100)
numnodes = 2000;
planner = mobileRobotPRM(map, numnodes);
planner.ConnectionDistance = 1;

waypoints = findpath(planner, startPosition, goalPosition);
Initializes a PRM planner and finds a path from startPosition to goalPosition.
Generating Trajectory
matlab
Copy code
robotheight = 0.12;
numWaypoints = size(waypoints,1);
firstInTime = 0;
lastInTime = firstInTime + (numWaypoints-1);
traj = waypointTrajectory(SampleRate=10, TimeOfArrival=firstInTime:lastInTime, Waypoints=[waypoints, robotheight*ones(numWaypoints,1)], ReferenceFrame="ENU");
Creates a trajectory for the robot with waypoints and timing information.
Loading and Setting Up the Robot
matlab
Copy code
huskyRobot = loadrobot("clearpathHusky");
platform = robotPlatform("husky", scenario, RigidBodyTree=huskyRobot, BaseTrajectory=traj);
Loads the Husky robot model and sets it up with the trajectory in the simulation environment.
Visualization
matlab
Copy code
[ax, plotFrames] = show3D(scenario);
lightangle(-45,30)
view(60,50)

hold(ax, "on")
plot(ax, waypoints(:,1), waypoints(:,2), "-ms", LineWidth=2, MarkerSize=4, MarkerEdgeColor="b", MarkerFaceColor=[0.5 0.5 0.5]);
hold(ax, "off")
Displays the environment and plots the waypoints in the 3D view.
Running the Simulation
matlab
Copy code
setup(scenario)
r = rateControl(20);
robotStartMoving = false;

while advance(scenario)
    show3D(scenario, Parent=ax, FastUpdate=true);
    waitfor(r);

    currentPose = read(platform);
    if ~any(isnan(currentPose))
        robotStartMoving = true;
    end
    if any(isnan(currentPose)) && robotStartMoving
        break;
    end
end

restart(scenario)
Runs the simulation, updating the scenario at 20 Hz. The simulation stops once the robot reaches the goal position.
Result
The script sets up a simulated environment with walls and obstacles, plans a path for the robot from a starting position to a goal position, and visualizes the robot’s movement through the environment. The environment and the robot’s trajectory are displayed in a 3D view, showing how the robot navigates through the obstacles.

Requirements
MATLAB
Robotics System Toolbox
Usage
Copy the script into a MATLAB environment.
Run the script to initialize the scenario and start the simulation.
Observe the robot navigating through the 3D environment in the MATLAB figure window.
License
This project is licensed under the MIT License - see the LICENSE file for details.

sql
Copy code

5. **Commit Your Changes:**
   - After pasting the content, scroll down to the "Commit changes" section.
   - Add a commit message (e.g., "Add initial README with simulation description").
   - Choose "Commit
