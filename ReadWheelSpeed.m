function dataWVel = ReadWheelSpeed(Robot)
% [vel_left_wheel, vel_right_wheel], rad/s
% vel_right_wheel/robot_yaw_rate ~= 0.05
latestWVel = receive(Robot.subWVel,3);
vlw = latestWVel.velocity_left;
vrw = latestWVel.velocity_right;
dataWVel = [vlw, vrw];
end