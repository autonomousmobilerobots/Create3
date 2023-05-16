%% Save IDs
names = ["Reserved","Murphy"];
IDs = [0,1];
nameROS2ID = dictionary(names,IDs);
save('Create3Names','nameROS2ID');

%% Testing Initialization
clear; clc
[Robot] = Create3Init("Murphy");
RID = Robot.ID
pause(1.0)
dataStore.Hzrd = [];
noRobotCount = 1;
[noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);