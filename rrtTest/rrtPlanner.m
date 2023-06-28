function[dataStore] = rrtPlanner(Robot,maxTime)
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
%     'odometry', [], ...
%     'rsdepth', [], ...
%     'bump', [], ...
%     'beacon', [], ...
%     'GPS', [], ...
%     'deadReck', [], ...
%     'gridOccBump', []);
dataStore = struct( 'truthPose', [],...
                    'Odom', [], ...
                    'Hzrd', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;
% parameters ---------------
forwardV = 0.5;
angularW = 0.1;
maxV = 0.1;
wheel2Center = 0.13;
radius =  0.16;
% Flags -------------------
initFlag = 0;
% Map
map = load('labBoxMap_wall_orange.mat').map;
goal1 = load('labBoxMap_wall_orange.mat').goal1;
xmin = min(map(:,[1,3]));
xmax = max(map(:,[1,3]));
ymin = min(map(:,[2,4]));
ymax = max(map(:,[2,4]));
omap = lins2omap(map,[xmin(1) xmax(1)],[ymin(1) ymax(1)]);
% Planner
goal = goal1;
contGain = -30.;
epsilon = 0.1;
curWptId = 1;
% TIC
tic
% plot stuff
clc
sg = 0.1;
figure()
pmap = plotOmap(omap);
% axis equal
while toc < maxTime
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    % Add noise to the GPS sensor
    state = dataStore.truthPose(end, 2:end);
    cmdV=0;
    cmdW=0;
    % Here is the planning section
    if ~initFlag
        start = state(1:2);
        [wpts] = buildRRT('labBoxMap_wall_orange',[xmin(1) ymin(1) xmax(1) ymax(1)],...
            start,goal,radius);
        pstt = plot(state(1),state(2),'r*');
        pgal = plot(goal(1),goal(2),'k*');
        treeEdges = load('rttTemp.mat').rttE;
        waypoints_startTOgoal = load('rttTemp.mat').waypoints_startTOgoal;
        ptre = plot(treeEdges(:,1:2)',treeEdges(:,3:4)','b--');
        ppth = plot(waypoints_startTOgoal(:,1),waypoints_startTOgoal(:,2),'r-',...
            'LineWidth',2);
        wpts = flip(wpts,1);
        initFlag = 1;
    else
        curWpt = wpts(curWptId,:);
        distLeft = norm([state(1) state(2)]-curWpt);
        if distLeft<0.1
            disp('Reach way point');
            curWptId = curWptId+1;
            if curWptId>length(wpts)
                disp('Reach Goal');
                break;
            end
        else
            cmdVec = contGain*([state(1) state(2)]-curWpt); % contGain < 0
            theta = state(3);
            [cmdV, cmdW] = feedbackLin(cmdVec(1),cmdVec(2),theta,epsilon);
            % plots
            pgrd = plot(state(1),state(2),'g*','MarkerSize',2);
        end
    end

    % Limit Velocity
    [cmdV, cmdW] = limitCmds(cmdV,cmdW,maxV,radius);
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
% set plot labels
xlabel('x position(m)');
ylabel('y position(m)');
legend([pgrd(1), pstt, pgal, ptre(1), ppth(1), pmap(1)],...
    'Trajectory','Start','Goal','Tree','Way points', 'Map walls');
end
