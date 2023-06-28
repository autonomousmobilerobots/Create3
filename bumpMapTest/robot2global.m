function[xyG] = robot2global(pose,xyR)
% ROBOT2GLOBAL: transform a 2D point in robot coordinates into global
% coordinates (assumes planar world).
% 
%   XYG = ROBOT2GLOBAL(POSE,XYR) returns the 2D point in global coordinates
%   corresponding to a 2D point in robot coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyR     2D point in robot coordinates (1-by-2)
% 
%   OUTPUTS
%       xyG     2D point in global coordinates (1-by-2)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   BO-RUEI HUANG
th = pose(3);
tx = pose(1);
ty = pose(2);
n = length(xyR(:, 1));
tbtf = [xyR ones(n,1)];
T_r2g = [cos(th),-sin(th), tx; sin(th), cos(th), ty; 0 0 1];
tfed = T_r2g*tbtf';
xyG = tfed(1:2, :);
xyG = xyG';
end

