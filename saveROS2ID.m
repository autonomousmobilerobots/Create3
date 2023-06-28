%% Save IDs
names = ["Reserved","Murphy"];
% domain ID, optitrack ID pairs
IDs = [0,1];
olIDs = ["","c3"];
nameROS2ID = dictionary(names,IDs);
nameoverID = dictionary(names,olIDs);
save('Create3Names','nameROS2ID','nameoverID');

%% Testing Initialization
clear; clc
[Robot] = Create3Init("Murphy");
RID = Robot.ID
pause(1.0)
dataStore.Hzrd = [];
noRobotCount = 1;
[noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);