%% Initialize
clear Robot
Robot = Create3Init('Murphy');

%% test beep
BeepRoomba(Robot)

%% test motor forward
travelDist(Robot, 0.1, -0.5)

%% test motor rotate
turnAngle(Robot, 0.2, -1.57)

%%
turnAngle(Robot, 0, 0)

%% test velocity
SetFwdVelAngVelCreate(Robot, 0.05, 0.05);
pause(2.0);
SetFwdVelAngVelCreate(Robot, 0.0, 0.0);

%% test light ring
% rgb = [255*eye(3);255*eye(3)];
rgb = [];
SetLightRingCreate(Robot, rgb);

%% test button
while true
buts = ReadButton(Robot)
pause(0.1);
end

%% test battery
while true
[voltage, percent, temp] = ReadBattery(Robot)
pause(0.1);
end

%% test IMU
while true
dataIMU = ReadIMU(Robot);
yaw = dataIMU(3)
pause(0.1);
end

%% test Wheel Velocity
while true
SetFwdVelAngVelCreate(Robot, 0.0, 0.05);
dataWVel = ReadWheelSpeed(Robot);
dataSlip = ReadWheelSlip(Robot)
% vrw = dataWVel(2)
pause(0.1);
end

%% backup bump
Robot = Create3Init('Murphy');
backupBump(Robot, 50);
