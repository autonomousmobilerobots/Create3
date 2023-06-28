function[dataStore] = backupBump(Robot,maxTime)
% TURNINPLACE: simple example program to use with iRobot Create (or simulator).
% Reads data from sensors, makes the robot turn in place and saves a datalog.
% 
%   dataStore = TURNINPLACE(Robot,maxTime) runs 
% 
%   INPUTStype
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%
% 	Modified: Liran 2023

% neglect all these
% 
% % Set unspecified inputs
% if nargin < 1
%     disp('ERROR: TCP/IP port object not provided.');
%     return;
% elseif nargin < 2
%     maxTime = 500;
% end
% 
% try 
%     % When running with the real robot, we need to define the appropriate 
%     % ports. This will fail when NOT connected to a physical robot 
%     CreatePort=Robot.CreatePort;
% catch
%     % If not real robot, then we are using the simulator object
%     CreatePort = Robot;
% end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct( 'truthPose', [],...
                    'Odom', [], ...
                    'Hzrd', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

SetFwdVelAngVelCreate(Robot, 0,0);
tic
% params
Vf = 0.08;
% Vt = 0.6; % Problem 1
Vt = 0.0;
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    % check bump
    isbump = dataStore.Hzrd(end,3);
    if isbump
        disp("Bump~");
        BeepRoomba(Robot);
        travelDist(Robot, Vf, -0.5);
        turnAngle(Robot, 0.1, 0.7);
    end

    % keep going~
    cmdV = Vf;
    cmdW = Vt;

    % Set angular velocity
    [cmdV,cmdW] = limitCmds(cmdV,cmdW,0.3,0.13);
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW );
    end
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0 );
