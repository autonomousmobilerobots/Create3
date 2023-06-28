function[dataStore] = mapBuilder(Robot,maxTime)
% -----------------------------------------------
% | <Instruction on switching the localization method>
% | At line 61~63, there are {ekfDepthFlag; ekfGPSFlag; pfDepthFlag},
% | these 3 variables can only be 1 or 0, and only one of them can be 1.
% | EKF using depth measurement: 
% | ekfDepthFlag=1; ekfGPSFlag=0; pfDepthFlag=0;
% | EKF using GPS measurement: 
% | ekfDepthFlag=0; ekfGPSFlag=1; pfDepthFlag=0;
% | Particle filter using depth measurement: 
% | ekfDepthFlag=0; ekfGPSFlag=0; pfDepthFlag=1;
% -----------------------------------------------
% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
% dataStore = struct('truthPose', [],...
%                    'odometry', [], ...
%                    'rsdepth', [], ...
%                    'bump', [], ...
%                    'beacon', [], ...
%                    'GPS', [], ...
%                    'deadReck', [], ...
%                    'ekfMu', [], ...
%                    'ekfSigma', [], ...
%                    'particles', [], ...
%                    'gridOccBump', []);
dataStore = struct( 'truthPose', [],...
                    'Odom', [], ...
                    'Hzrd', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;
% parameters ---------------
forwardV = 0.1;
angularW = 0.1;
maxV = 0.3;
wheel2Center = 0.13;
n_rs_rays = 9;
sensor_pos = [0 0.08];
radius =  0.16;
% Flags -------------------
initFlag = 0;
bumpMapFlag = 1;
depthMapFlag = 0;
% Map
p.n = 100; % num of cells in X dimension
p.m = 100; % num of cells in Y dimension
p.mapRngX = [-3;0]; % map range in X [xmin;xmax]
p.mapRngY = [-3;0]; % map range in Y [ymin;ymax]
rng = 27; nB = 9;
p.beamAngle = -(-rng:(2*rng/nB):rng)*pi/180; % depth sensor beam angles from left to right (pos->neg) [1*nB]
p.sensorLoc = [0;0.08]; % sensor location in body frame [2*1]
p.sigma = [0.03;0.03]*ones(1,nB); % depth information uncertainties in Body frame [2*nB]
p.bumpPose = [radius;...
                   0;...
                   0]; 
p.l0 = 0;
% sX = (p.mapRngX(2)-p.mapRngX(1))/p.n'*1;
% sY = (p.mapRngY(2)-p.mapRngY(1))/p.m'*5;
% p.sigma = [sX sX sX; sY sY sY];
% cangs = -(pi):(2*pi/8):(pi);
cangs = (pi/2):(2*pi/8):(3*pi/2);
p.robCont = 0.2*radius*[cos(cangs);sin(cangs)]; % Robot contour points in body frame [2*nC]
gridMap = 0.5*ones(p.n,p.m);
plotThres = [-0.2,0.75];
% TIC
tic
% plot stuff
clc
figure()
hold on
axis equal
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    state = dataStore.truthPose(end, 2:end);
    
    % CONTROL FUNCTION (send robot commands)
    tbump = dataStore.Hzrd(end,1);
    bumps = dataStore.Hzrd(end,3);

    [cmdV,cmdW] = limitCmds(forwardV,angularW,maxV,wheel2Center);
    if sum(bumps) >= 1
        d = -0.25;
        theta = -30*3.14/180;
        disp("Bump~");
        BeepRoomba(Robot);
        travelDist(Robot, cmdV, d);
        turnAngle(Robot, cmdV, theta);
    end
    
    % Here is the mapping section
    if ~initFlag
        % Initialize pose for navigations
        pmap = plotOccupancyGrid(p,gridMap,plotThres);
        pgrd = plot(state(1),state(2),'ro');
        initFlag = 1;
    elseif bumpMapFlag
        gridMap = logOddsBump(gridMap, state', bumps', p);
        % Plot
        delete(pgrd)
        delete(pmap)
        pmap = plotOccupancyGrid(p,gridMap,plotThres);
        pgrd = plot(state(1),state(2),'ro');
    end

    % Limit Velocity
    [cmdV, cmdW] = limitCmds(cmdV,cmdW,0.5,radius);
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW );
    end
    pause(0.01);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0 );
end
