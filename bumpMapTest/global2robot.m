function[xyR] = global2robot(pose,xyG)
% GLOBAL2ROBOT: transform a 2D point in global coordinates into robot
% coordinates (assumes planar world).
% 
%   XYR = GLOBAL2ROBOT(POSE,XYG) returns the 2D point in robot coordinates
%   corresponding to a 2D point in global coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyG     2D point in global coordinates (1-by-2)
% 
%   OUTPUTS
%       xyR     2D point in robot coordinates (1-by-2)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   BO-RUEI, HUANG
th = pose(3);
tx = pose(1);
ty = pose(2);
n = length(xyG(:, 1));
tbtf = [xyG ones(n,1)];
a = -cos(th)*tx - sin(th)*ty;
b =  sin(th)*tx - cos(th)*ty;
T_g2r = [cos(th), sin(th), a;-sin(th), cos(th), b; 0 0 1];
tfed = T_g2r*tbtf';
xyR = tfed(1:2, :);
xyR = xyR';
end
