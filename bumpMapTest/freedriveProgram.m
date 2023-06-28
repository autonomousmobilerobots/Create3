function[dataStore] = freedriveProgram(Robot,maxTime)
% freedriveProgram: example program to manually drive iRobot Create
% Reads data from sensors, sends robot commands, and saves a datalog.
% 
%   dataStore = freedriveProgram(Robot,maxTime) 
% 
%   INPUTS
%       Robot       Robot structure (created by CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data
% 
%   Cornell University
%   AMR: Autonomous Mobile Robots
%   Lab #3
%   Modified by Liran 2021

% Set unspecified inputs
if nargin < 1
    disp('ERROR: Robot input not provided');
    return;
elseif nargin < 2
    maxTime = 1000;
end

% Call up manual drive GUI
h = driveArrows(Robot);

% declare datastore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'GPS', [], ...
                   'gridOccBump', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;
% parameters ---------------
forwardV = 0.5;
angularW = 0.1;
maxV = 1.8;
wheel2Center = 0.13;
n_rs_rays = 9;
sensor_pos = [0 0.08];
radius =  0.16;
% Flags -------------------
initFlag = 0;
bumpMapFlag  = 0;
depthMapFlag = 1;
% Map
p.mapRngX = [-3.5;3.5]; % map range in X [xmin;xmax]
p.mapRngY = [-2.5;2.5]; % map range in Y [ymin;ymax]
p.l0 = 0;
if depthMapFlag
    p.n = 22; % num of cells in X dimension
    p.m = 12; % num of cells in Y dimension
    rng = 27; nB = 9;
    p.beamAngle = -(-rng:(2*rng/nB):rng)*pi/180; % depth sensor beam angles from left to right (pos->neg) [1*nB]
    p.sensorLoc = [0;0.08]; % sensor location in body frame [2*1]
    p.sigma = [0.03;0.03]*ones(1,nB); % depth information uncertainties in Body frame [2*nB]
    cangs = (pi/2):(2*pi/8):(3*pi/2);
elseif bumpMapFlag
    p.n = 16; % num of cells in X dimension
    p.m = 28; % num of cells in Y dimension
    p.bumpPose = [radius/1.4 radius   radius/1.4 ;...
                  radius/1.4      0  -radius/1.4 ;...
                        pi/4      0         -pi/4]; 
    sX = (p.mapRngX(2)-p.mapRngX(1))/p.n'*0.5;
    sY = (p.mapRngY(2)-p.mapRngY(1))/p.m'*5;
    p.sigma = [sX sX sX; sY sY sY];
    cangs = (pi/2):(2*pi/8):(3*pi/2);
% cangs = -(pi):(2*pi/8):(pi);
end
p.robCont = 0.2*radius*[cos(cangs);sin(cangs)]; % Robot contour points in body frame [2*nC]
gridMap = 0.5*ones(p.n,p.m);
plotThres = [-0.2,0.75];
% TIC
tic
% plot stuff
clc
% f1 = figure();
hold on
axis equal
while toc<maxTime
    
    % Read and Store Sensore Data
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);

    % Add noise to the GPS sensor
    state = dataStore.truthPose(end, 2:end);
    sstate = size(state);
    noiseGPS = normrnd(state, 0.0316*ones(sstate(1),sstate(2)));
    dataStore.GPS = [dataStore.GPS; noiseGPS];

    % Extract depth data
    zDepth = dataStore.rsdepth(end, 3:end);
    
    % CONTROL FUNCTION (send robot commands)
    tbump = dataStore.bump(end,1);
    bumps = dataStore.bump(end,[3,7,2]);

    % [cmdV,cmdW] = limitCmds(forwardV,angularW,maxV,wheel2Center);
    % if sum(bumps) >= 1
    %     d = -0.25;
    %     theta = -30;
    %     TravelDistCreate(Robot, cmdV, d)
    %     TurnCreate(Robot, cmdV, theta);
    % end
    
    % Update Occupancy Grids Here with logOddsBump.m or logOddsDepth.m 
    figure(h);
    if ~initFlag
        % Initialize pose for navigations
        pmap = plotOccupancyGrid(p,gridMap,plotThres);
        pgrd = plot(state(1),state(2),'ro');
        ptrj = plot(dataStore.truthPose(1:end, 2),dataStore.truthPose(1:end, 3),'b-','LineWidth',2);
        initFlag = 1;
    elseif bumpMapFlag
        gridMap = logOddsBump(gridMap, state', bumps', p);
    % Plot Occupancy Grid and robot trajectory in real time 
        delete(pgrd)
        delete(pmap)
        delete(ptrj)
        pmap = plotOccupancyGrid(p,gridMap,plotThres);
        pgrd = plot(state(1),state(2),'ro');
        ptrj = plot(dataStore.truthPose(1:end, 2),dataStore.truthPose(1:end, 3),'b-','LineWidth',2);
    elseif depthMapFlag
        gridMap = logOddsDepth(gridMap, state', zDepth', p);
    % Plot Occupancy Grid and robot trajectory in real time 
        delete(pgrd)
        delete(pmap)
        delete(ptrj)
        pmap = plotOccupancyGrid(p,gridMap,plotThres);
        pgrd = plot(state(1),state(2),'ro');
        ptrj = plot(dataStore.truthPose(1:end, 2),dataStore.truthPose(1:end, 3),'b-','LineWidth',2);
    end

    % Make sure that the GUI for keyboard contorl is in focus
    figure(h);
    
    pause(0.1);
end
% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0 );

