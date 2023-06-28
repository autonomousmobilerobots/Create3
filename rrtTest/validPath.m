function [isVal] = validPath(walls,radius,startV,distance,angle)
% walls   : [4xnW]
% radius  : [meter] the radius of a robot
% startV  : [2x1]
% distance: [meter] the distance between q and qnew
% angle   : [rad]   the global frame angle between q and qnew
%
% [1] build the bounding box of the q-qnew line
A = startV+radius*[-sin(angle);cos(angle)];
B = A+(distance+radius)*[cos(angle);sin(angle)];
D = startV-radius*[-sin(angle);cos(angle)];
C = D+(distance+radius)*[cos(angle);sin(angle)];
bbWall = [[A;B] [B;C] [C;D]];
% pbbx = plot([A(1) B(1) C(1) D(1)],[A(2) B(2) C(2) D(2)],'r-');
nBW = size(bbWall,2);
nW = size(walls,2);
% [2] check for the intersection between walls and bound box
isVal = true;
for i=1:nBW
    bi=bbWall(:,i);
    for j=1:nW
        wj=walls(:,j);
        [isInt,~,~] = intersectPoint(bi(1),bi(2),bi(3),bi(4),...
            wj(1),wj(2),wj(3),wj(4));
        if isInt
            % disp('invalid path')
            isVal = false;
            break;
        end
    end
end
end