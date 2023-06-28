function [finalPose] = integrateOdom(initPose,d,phi)
% integrateOdom: Calculate the robot pose in the initial frame based on the
% odometry
% 
% [finalPose] = integrateOdom(initPose,dis,phi) returns a 3-by-N matrix of the
% robot pose in the initial frame, consisting of x, y, and theta.

%   INPUTS
%       initPose    robot's initial pose [x y theta]  (3-by-1)
%       d     distance vectors returned by DistanceSensorRoomba (1-by-N)
%       phi     angle vectors returned by AngleSensorRoomba (1-by-N)

% 
%   OUTPUTS
%       finalPose     The final pose of the robot in the initial frame
%       (3-by-N)

%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
pi = 3.14;
n  = length(d);
finalPose = zeros(3, n);
tempPose = initPose;
for i = 1:n
    nx = tempPose(1)+d(i)*sinc(phi(i)/2/pi)*cos(tempPose(3)+phi(i)/2);
    ny = tempPose(2)+d(i)*sinc(phi(i)/2/pi)*sin(tempPose(3)+phi(i)/2);
    nt = tempPose(3)+phi(i);
    finalPose(:,i) = [nx; ny; nt];
    tempPose = [nx; ny; nt];
end
end

