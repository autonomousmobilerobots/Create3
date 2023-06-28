function [Robot] = Create3Init(RobotName)
%CREATE3INIT Summary of this function goes here
%   Detailed explanation goes here
if nargin<1
	error('Missing remoteHost argument.  See help CreatePiInit'); 
end
% ===<Initialize ROS2 Node>===
nameROS2ID = load("Create3Names").nameROS2ID;
Robot.ID = nameROS2ID(RobotName);
Robot.hostNodeName = 'hostNode';
Robot.hostNode = ros2node(Robot.hostNodeName,Robot.ID);
pause(0.6);
% ===<Set Default Domain ID for Command Line Util>===
setenv("ROS_DOMAIN_ID",string(Robot.ID));
% ===<Check Battery and Initialize Epoch Time>===
% Quit... The pub. freq. is too low
% Robot.subBatt = ros2subscriber(Robot.hostNode,"/battery_state",...
%     "Reliability","besteffort");
% latestBatt = receive(Robot.subBatt,10);
% ===<Initialize ROS2 Pub.>===
Robot.pubLRng = ros2publisher(Robot.hostNode,"/cmd_lightring",...
    "Reliability","besteffort");
Robot.pubSund = ros2publisher(Robot.hostNode,"/cmd_audio");
Robot.pubLAVL = ros2publisher(Robot.hostNode,"/cmd_vel",...
    "Reliability","besteffort");
% ===<Initialize ROS2 Sub.>===
Robot.subHzrd = ros2subscriber(Robot.hostNode,"/hazard_detection",...
    "Reliability","besteffort");
Robot.subOdom = ros2subscriber(Robot.hostNode,"/odom",...
    "Reliability","besteffort");
Robot.subWVel = ros2subscriber(Robot.hostNode,"/wheel_vels",...
    "Reliability","besteffort");
Robot.subWSlp = ros2subscriber(Robot.hostNode,"/slip_status",...
    "Reliability","besteffort");
Robot.subInMU = ros2subscriber(Robot.hostNode,"/imu",...
    "Reliability","besteffort");
Robot.subBttn = ros2subscriber(Robot.hostNode,"/interface_buttons",...
    "Reliability","besteffort");
Robot.subBtry = ros2subscriber(Robot.hostNode,"/battery_state",...
    "Reliability","besteffort");
% A test for subscriber callback, not used
% Robot.subHzrd = ros2subscriber(Robot.hostNode,"/hazard_detection",...
    % @subHzrdCallback,...
    % "Reliability","besteffort");
%
% ===<Initialize Epoch Time>===
latestHzrd = receive(Robot.subHzrd,3);
Robot.epochROS2 = ros2StampToFloat(latestHzrd.header.stamp,0);
% disp('Epoch Time: ');
% disp(Robot.epochROS2);
% ===<Initialize Opti-Track>===
Robot.olClient = Init_OverheadLocClient();
nameoverID = load("Create3Names").nameoverID;
Robot.olID = nameoverID(RobotName);
end

% function subHzrdCallback(message)
% disp(message)
% end
