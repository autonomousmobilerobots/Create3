function dataSlip = ReadWheelSlip(Robot)
% dataSlip=1 --> the robot slips
latestWSlp = receive(Robot.subWSlp,3);
dataSlip = latestWSlp.is_slipping;
end