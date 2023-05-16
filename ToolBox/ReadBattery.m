function [voltage, percent, temperature]  = ReadBattery(Robot)
latestBtry = Robot.subBtry.LatestMessage;
voltage = latestBtry.voltage;
percent = latestBtry.percentage;
temperature = latestBtry.temperature;
end