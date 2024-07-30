scenario = robotScenario(UpdateRate=5);

floorColor = [0.5882 0.2941 0];
addMesh(scenario,"Plane",Position=[5 4 0],Size=[10 10],Color=floorColor);

wallHeight = 2;
wallWidth = 0.22;
wallLength = 10;
wallColor = [1 1 0.8157];

% Add outer walls.
addMesh(scenario,"Box",Position=[wallWidth/2, wallLength/2, wallHeight/2],...
    Size=[wallWidth, wallLength, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallLength-wallWidth/2, wallLength/2, wallHeight/2],...
    Size=[wallWidth, wallLength, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallLength/2, wallLength-wallWidth/2, wallHeight/2],...
    Size=[wallLength, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallLength/2, wallWidth/2, wallHeight/2],...
    Size=[wallLength, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true);

% Add inner walls.
addMesh(scenario,"Box",Position=[wallLength/8, wallLength/3, wallHeight/2],...
    Size=[wallLength/4, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallLength/4, wallLength/3, wallHeight/2],...
    Size=[wallWidth, wallLength/6,  wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[(wallLength-wallLength/4), wallLength/2, wallHeight/2],...
   Size=[wallLength/2, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallLength/2, wallLength/2, wallHeight/2],...
    Size=[wallWidth, wallLength/3, wallHeight],Color=wallColor,IsBinaryOccupied=true);

show3D(scenario);
lightangle(-45,30);
view(30,60);

map = binaryOccupancyMap(scenario,GridOriginInLocal=[-2 -2],...
                                           MapSize=[15 15],...
                                           MapHeightLimits=[0 3]);
inflate(map,0.3);

show(map)

startPosition = [1 2];
goalPosition = [8 8];

rng(100)

numnodes = 2000;
planner = mobileRobotPRM(map,numnodes);
planner.ConnectionDistance = 1;

waypoints = findpath(planner,startPosition,goalPosition);

% Robot height from base.
robotheight = 0.12;
% Number of waypoints.
numWaypoints = size(waypoints,1);
% Robot arrival time at first waypoint.
firstInTime = 0;
% Robot arrival time at last waypoint.
lastInTime = firstInTime + (numWaypoints-1);
% Generate waypoint trajectory with waypoints from planned path.
traj = waypointTrajectory(SampleRate=10,...
                          TimeOfArrival=firstInTime:lastInTime, ...
                          Waypoints=[waypoints, robotheight*ones(numWaypoints,1)], ...
                          ReferenceFrame="ENU");

huskyRobot = loadrobot("clearpathHusky");

platform = robotPlatform("husky",scenario, RigidBodyTree=huskyRobot,...
                         BaseTrajectory=traj);

[ax,plotFrames] = show3D(scenario);
lightangle(-45,30)
view(60,50)

hold(ax,"on")
plot(ax,waypoints(:,1),waypoints(:,2),"-ms",...
               LineWidth=2,...
               MarkerSize=4,...
               MarkerEdgeColor="b",...
               MarkerFaceColor=[0.5 0.5 0.5]);
hold(ax,"off")

setup(scenario)

% Control simulation rate at 20 Hz.
r = rateControl(20);

% Status of robot in simulation.
robotStartMoving = false;

while advance(scenario)
    show3D(scenario,Parent=ax,FastUpdate=true);
    waitfor(r);

    currentPose = read(platform);
    if ~any(isnan(currentPose))
        % implies that robot is in the scene and performing simulation.
        robotStartMoving = true;
    end
    if any(isnan(currentPose)) && robotStartMoving
        % break, once robot reaches goal position.
        break;
    end
end

restart(scenario)