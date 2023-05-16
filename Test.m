%% Test on importing Create3 messages
clc; clear
folderPath = fullfile(pwd,"irobot_create_msgs");
ros2genmsg(pwd);

% if the workspace changes place, the message should be rebult.

%% Receiving Messages
clear defaultNode
defaultNode = ros2node('default',1);

%%
clear sub
subwt = ros2subscriber(defaultNode,"/wheel_ticks","Reliability","besteffort"); % ,"irobot_create_msgs/WheelTicks");
% if the message type is provided, the subscriber can be created without discovering corresponding topics among the network
msswt = receive(subwt,3);

%% Odom test
clear subid; clear mssod
subod = ros2subscriber(defaultNode,"/odom",...
    "Reliability","besteffort");
mssod = receive(subod,3);

%%
clear subtf
subtf = ros2subscriber(defaultNode,"/tf");
lmstf = subtf.LatestMessage;
msstf = receive(subtf,10);

%%
clear suimu
suimu = ros2subscriber(defaultNode,"/imu","Reliability","besteffort");
lmsim = suimu.LatestMessage;
mssim = receive(suimu,10);

%%
subws = ros2subscriber(defaultNode,"/wheel_status","Reliability","besteffort"); % ,"irobot_create_msgs/WheelTicks");
% if the message type is provided, the subscriber can be created without discovering corresponding topics among the network
mssws = receive(subws,3);

%%
subwv = ros2subscriber(defaultNode,"/wheel_vels","Reliability","besteffort"); % ,"irobot_create_msgs/WheelTicks");
% if the message type is provided, the subscriber can be created without discovering corresponding topics among the network
msswv = receive(subwv,3);

%% 
subss = ros2subscriber(defaultNode,"/slip_status","Reliability","besteffort"); % ,"irobot_create_msgs/WheelTicks");
% if the message type is provided, the subscriber can be created without discovering corresponding topics among the network
mssss = receive(subss,3);

%%
clear subst mssst
subst = ros2subscriber(defaultNode,"/stop_status",...
    "Reliability","besteffort","Durability","transientlocal");
% if the message type is provided, the subscriber can be created without discovering corresponding topics among the network
mssst = receive(subst,10);

%% {'/cmd_vel'} | {'geometry_msgs/Twist'}
clear twist cmdPub
twist = ros2message("geometry_msgs/Twist");
twist.angular.z = 0.1;
cmdPub = ros2publisher(defaultNode,"/cmd_vel","geometry_msgs/Twist","Reliability","besteffort");
send(cmdPub,twist)

%% /cmd_audio irobot_create_msgs/msg/AudioNoteVector
% "{append: false, 
% notes: [{frequency: 392, max_runtime: {sec: 0,nanosec: 177500000}},
% {frequency: 523, max_runtime: {sec: 0,nanosec: 355000000}}, {frequency:
% 587, max_runtime: {sec: 0,nanosec: 177500000}}, {frequency: 784,
% max_runtime: {sec: 0,nanosec: 533000000}}]}" -1
clear sound soundPub
sound = ros2message("irobot_create_msgs/AudioNoteVector");
sound.notes.frequency = uint16(392);
sound.notes.max_runtime.sec = int32(0);
sound.notes.max_runtime.nanosec = uint32(177500000);
soundPub = ros2publisher(defaultNode,"/cmd_audio","irobot_create_msgs/AudioNoteVector");
send(soundPub,sound)

%%
clear publr
publr = ros2publisher(defaultNode,"/cmd_lightring"); %,"Reliability","besteffort","Durability","transientlocal");
msslr = ros2message("irobot_create_msgs/LightringLeds");
% msslr.override_system = true;
% col.red = uint8(255);
% col.green = uint8(0);
% col.blue = uint8(0);
% msslr.leds = [col,col,col,col,col,col];
send(publr, msslr);

%% Problem Met so far
%  1. Path Length may be too long->msg build could faild
%  2. Visual C++ 14 or greater required
%  3. ROS toolbox preference only works on 2023a