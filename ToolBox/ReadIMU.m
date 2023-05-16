function dataIMU = ReadIMU(Robot)
% dataIMU: [1x9] [roll, pitch, yaw, wx, wy, wz, ax, ay, az]
latestIMU = receive(Robot.subInMU,3);
% Compass
cw = latestIMU.orientation.w;
cx = latestIMU.orientation.x;
cy = latestIMU.orientation.y;
cz = latestIMU.orientation.z;
[yaw, pitch, roll] = quaternion2EulerBodyZYX(cw,cx,cy,cz);
% Angular Rate
wx = latestIMU.angular_velocity.x;
wy = latestIMU.angular_velocity.y;
wz = latestIMU.angular_velocity.z;
% Linear Acceleration
ax = latestIMU.angular_velocity.x;
ay = latestIMU.angular_velocity.y;
az = latestIMU.angular_velocity.z;
% compose the output
dataIMU = [roll pitch yaw wx wy wz ax ay az];
end