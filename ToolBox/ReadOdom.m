function dataOdom = ReadOdom(Robot)
% ===<Odometry>===
% use a persistance variable to store the last pose
persistent lastPose
if isempty(lastPose)
    lastPose = [0 0 0];
end
% start doing odom calculation
latestOdom = receive(Robot.subOdom,3);
% compose current Odometry pose
currPoseX = latestOdom.pose.pose.position.x;
currPoseY = latestOdom.pose.pose.position.y;
currOrien = latestOdom.pose.pose.orientation;
[currTheta, ~, ~] = quaternion2EulerBodyZYX(currOrien.w,currOrien.x,currOrien.y,currOrien.z);
currPose = [currPoseX currPoseY currTheta];
% calculate the odometry
dataOdom = pose2Odom(currPose, lastPose);
lastPose = currPose;
end

function u = pose2Odom(currPose, lastPose)
phi = real(log(exp(1i*currPose(3))/exp(1i*lastPose(3)))/(1i));
% determine the distance traveled
dx = currPose(1)-lastPose(1);
dy = currPose(2)-lastPose(2);
d = norm([dx dy])/sinc(phi/2/pi);
u = [d, phi];
end

function y = sinc(x)
if x==0
    y = 1;
else
    y = sin(x*pi)/(x*pi);
end
end
