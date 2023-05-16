function dataHzrd = ReadHazard(Robot)
% ===<Hazard>===
% Definition:
% https://github.com/iRobotEducation/irobot_create_msgs/blob/rolling/msg/HazardDetection.msg
% BACKUP_LIMIT     0: reach backup limit.  The limit is over write by default
% BUMP             1: front bump sensor triggered
% CLIFF            2: cliff detected
% STALL            3: stalled against an obstacle
% WHEEL_DROP       4: wheels completely leave the ground
% OBJECT_PROXIMITY 5: close to an obstacle
latestHzrd = receive(Robot.subHzrd,3);
% if we are using ROS2 time.  Not the case here.
% timeHzrd = ros2StampToFloat(latestHzrd.header.stamp,Robot.epochROS2);
dettHzrd = latestHzrd.detections;
dataHzrd = zeros(1,6);
if ~isempty(dettHzrd)
    sizeHzrd = size(dettHzrd, 1);
    for i=1:sizeHzrd
        typei = dettHzrd.type;
        if (typei>=0)&&(typei<=5)
            dataHzrd(typei+1) = 1;
        else
            msgStr = "Unknown Hazard Type: " + string(typei);
            disp(msgStr);
        end
    end
end
end