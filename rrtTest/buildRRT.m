function [waypoints_startTOgoal] = buildRRT(map,mapBoundary,start,goal,radius)
% BUILDRRTPOINT
% AMR Homework 6
%
%       INPUTS:
%           map             file name for text file representing the obstacles in the workspace
%                           for example map = 'hw6b.txt'. Each row in this file contains the vertices
%                           of one polygonal obstacle: v1x, v1y, v2x, v2y, etc. The vertices are given in
%                           counterclockwise order. If an obstacle has fewer vertices, unused entries
%                           in the line will contain the value zero.
%           mapBoundary     1 x 4 vector capturing the [x_bl y_bl x_tr y_tr] coordinates of the bottom left
%                           and top right corner of the workspace respectively
%           start           1 x 2 array [x y], for start point
%           goal            1 x 2 array [x y], for goal point
%           radius          scalar [meter]
%       OUTPUTS:
%           waypoints       n x 2 array, for a series of waypoints (n waypoints) defining a
%                           collision-free path from start to goal
% Autonomous Mobile Robots
%
% [0] parameters
step = 0.6; % meter
% [1] build omap
load(map); %[4x16]
omap = lins2omap(map,mapBoundary([1,3]),mapBoundary([2,4]));
nP = length(omap.polys);
walls = omap.walls;
% start and goal have to be 2x1 in the following section
start = start';
goal  = goal';
% [2] do RRT
Queu = [start]; % [2xnQ]
lstV = [-1];    % [1xnQ]
% pqnr = plot(start(1),start(2),'b*');
% pqnr = plot(goal(1),goal(2),'r*');
rttE = [];
notRch = true;
treeBnd = mapBoundary;
offX = 0.5*step; offY = offX;
while notRch
    % [2a] get qrand and qnear
    qrandFound = 0;
    qrand = [0;0];
    % bound the qrandi
    treeBnd = [min(Queu(1,:)) min(Queu(2,:)) max(Queu(1,:)) max(Queu(2,:))];
    Dx = treeBnd(3)-treeBnd(1)+2*offX;
    Dy = treeBnd(4)-treeBnd(2)+2*offY;
    while ~qrandFound
        % qrandi = treeBnd(1:2)'+[rand()*Dx;rand()*Dy]-[offX;offY]; % [2*1]
        inMap = false;
        qrandi = [0;0];
        while ~inMap
            randis = normrnd(sqrt(Dx^2+Dy^2)/2,0.8*step);
            ranang = -pi+2*pi*(rand());
            qrandi = start+randis*[cos(ranang);sin(ranang)];
            if (mapBoundary(1)<qrandi(1)) && (mapBoundary(2)<qrandi(2))...
                    && (mapBoundary(3)>qrandi(1)) && (mapBoundary(4)>qrandi(2))
                inMap = true;
            end
        end
        % pqri = plot(qrandi(1),qrandi(2),'ro');
        % pause(0.05);
        notIn = 1;
        for j=1:nP
            if checkInsidePoly(omap.polys(j).vid,qrandi,omap.vs,omap.ids,200)
                notIn = 0;
                break;
            end
        end
        if notIn
            qrand = qrandi;
            qrandFound = 1;
            break;
        end
        % delete(pqri);
    end
    dQues = Queu-qrand;
    distQ = vecnorm(dQues,2,1); % [1xnQ]
    iqnear = find(distQ==min(distQ));
    qnear = Queu(:,iqnear(1));
    % pqnr = plot(qnear(1),qnear(2),'b*');
    % [2b] get qnew
    delnr = qrand-qnear;
    disnr = norm(delnr);
    disnn = min(step,disnr); % qnear to qnew
    qnew  = qnear+(disnn/disnr)*(qrand-qnear);
    % pqnw = plot(qnew(1),qnew(2),'bs');
    delnn = qnew-qnear;
    angnn = atan2(delnn(2),delnn(1));
    % [2c] check valid path to qnew
    isV = validPath(walls,radius,qnear,disnn,angnn);
    if isV
        Queu = [Queu qnew];
        lstV = [lstV iqnear];
        % pbrh = plot([qnear(1) qnew(1)],[qnear(2) qnew(2)],'b--');
        rttE = [rttE;[qnear(1) qnew(1) qnear(2) qnew(2)]];
        % [2d] check if qnew to the goal is valid
        delng = goal-qnew;
        angng = atan2(delng(2),delng(1));
        disng = norm(delng);
        isV = validPath(walls,radius,qnew,disng,angng);
        if isV
            Queu = [Queu goal];
            lstV = [lstV (size(Queu,2)-1)];
            % pfnb = plot([qnew(1) goal(1)],[qnew(2) goal(2)],'b--');
            rttE = [rttE;[qnew(1) goal(1) qnew(2) goal(2)]];
            notRch = false;
        end
    end
    % pause(0.01);
    % delete(pqnr);
    % delete(pqnw);
end
% [3] rebuild the whole path
lastiQ = length(Queu);
path = [];
while lastiQ ~= -1
    path = [path Queu(:,lastiQ)];
    lastiQ = lstV(lastiQ);
end
% pwpt = plot(path(1,:),path(2,:),'r-','LineWidth',2);
waypoints_startTOgoal = path';
% for graph:
% legend([pbrh,pwpt],'RRT','Path');
save('rttTemp','rttE','Queu','lstV','waypoints_startTOgoal','start','goal');
end
