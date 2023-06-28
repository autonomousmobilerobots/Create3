function [cmdV, cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon)
% FEEDBACKLIN Transforms Vx and Vy commands into V and omega commands using
% feedback linearization techniques
% Inputs:
%  cmdVx: input velocity in x direction wrt inertial frame
%  cmdVy: input velocity in y direction wrt inertial frame
%  theta: orientation of the robot
%  epsilon: turn radius
% Outputs:
%  cmdV: fwd velocity
%  cmdW: angular velocity
e = epsilon;
vw = [1 0; 0 1/e]*[cos(theta) sin(theta); -sin(theta) cos(theta)]*[cmdVx; cmdVy];
cmdV = vw(1);
cmdW = vw(2);
end