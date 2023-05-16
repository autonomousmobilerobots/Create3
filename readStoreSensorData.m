function [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore)
% ===<Hazard>===
dataHzrd = nan(1,6);
try
    dataHzrd = ReadHazard(Robot);
catch 
    disp("Hazard Subscription Error");
    % rethrow(ME);
end
dataStore.Hzrd = [dataStore.Hzrd;[toc dataHzrd]];
% ===<Odometry>===
dataOdom = nan(1,2);
try
    dataOdom = ReadOdom(Robot);
catch 
    disp("Odometry Subscription Error");
    % rethrow(ME);
end
dataStore.Odom = [dataStore.Odom;[toc dataOdom]];
end

function data = lastValid(stream)
data = [];
nS = size(stream,1);
for i=1:nS
    datai = stream(end-i+1,:);
    if ~isnan(datai(2:end))
        data = datai;
        break;
    end
end
end
