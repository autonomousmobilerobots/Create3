%% plotBumpGrid
clc; clear
% load
load('gridBump_1')
% define data
time = dataStore.truthPose(:,1);
rtrj = dataStore.truthPose(:,2:end);
bump = dataStore.bump(:,[3,7,2]);
% load maps
map = load('loopMap.mat').loopMap;

%% Params
n_rs_rays = 9;
sensor_pos = [0 0.08];
radius =  0.16;
p.n = 25; % num of cells in X dimension
p.m = 25; % num of cells in Y dimension
p.mapRngX = [min([map(:,1);map(:,3)]);max([map(:,1);map(:,3)])]; % map range in X [xmin;xmax]
p.mapRngY = [min([map(:,2);map(:,4)]);max([map(:,2);map(:,4)])]; % map range in Y [ymin;ymax]
p.bumpPose = [radius/1.4 radius   radius/1.4 ;...
              radius/1.4      0  -radius/1.4 ;...
                    pi/4      0         -pi/4]; 
sX = (p.mapRngX(2)-p.mapRngX(1))/p.n'*1;
sY = (p.mapRngY(2)-p.mapRngY(1))/p.m'*5;
p.sigma = [sX sX sX; sY sY sY];
% cangs = -(pi):(2*pi/8):(pi);
cangs = (pi/2):(2*pi/8):(3*pi/2);
p.robCont = 0.2*radius*[cos(cangs);sin(cangs)]; % Robot contour points in body frame [2*nC]
p.l0 = 0;
gridMap = 0.5*ones(p.n,p.m);
plotThres = [-0.2,0.75];

%% Rerun and Fast Plot
clc
L = length(time);
bumpTrigg = [];
p.n = 25; % num of cells in X dimension
p.m = 25; % num of cells in Y dimension
rng = 1:L;
gridMap = 0.5*ones(p.n,p.m);
for i=rng
    state = rtrj(i,:);
    bumps = bump(i,:);
    gridMap = logOddsBump(gridMap, state', bumps', p);
    % if bumped
    if sum(bumps) >= 1
        bumpTrigg = [bumpTrigg;state];
    end
end
%% plot
figure()
hold on
pgrd = plotOccupancyGrid(p,gridMap,plotThres);
ppos = plot(rtrj(rng,1),rtrj(rng,2),'b-','LineWidth', 1.5);
pbps = plot(bumpTrigg(:,1),bumpTrigg(:,2),'r*');
pmap = plot([map(:,1) map(:,3)]', [map(:,2) map(:,4)]', 'g-', 'LineWidth', 2);
title('Bump sensor grid map');
xlabel('x position (m)');
ylabel('y position (m)');
legend([pgrd ppos pbps pmap(1)],'Grid Map','True trajectory','Bump trigger position','Wall');
% legend([pgrd ppos pmap(1)],'Grid Map','True trajectory','Wall');
axis equal
hold off

%% Plot triggered location
figure()
hold on
pgrd = plotOccupancyGrid(p,-1*ones(p.n,p.m),plotThres);
ppos = plot(rtrj(rng,1),rtrj(rng,2),'b-','LineWidth', 1.5);
pbps = plot(bumpTrigg(:,1),bumpTrigg(:,2),'r*');
pmap = plot([map(:,1) map(:,3)]', [map(:,2) map(:,4)]', 'g-', 'LineWidth', 2);
title('Bump sensor triggered position');
xlabel('x position (m)');
ylabel('y position (m)');
legend([pgrd ppos pmap(1)],'Grids','True trajectory','Wall');
axis equal
hold off

%% Plot black white
figure()
hold on
plotThres = [0.2;0.2];
pgrd = plotOccupancyGrid(p,gridMap,plotThres);
ppos = plot(rtrj(rng,1),rtrj(rng,2),'b-','LineWidth', 1.5);
pbps = plot(bumpTrigg(:,1),bumpTrigg(:,2),'r*');
pmap = plot([map(:,1) map(:,3)]', [map(:,2) map(:,4)]', 'g-', 'LineWidth', 2);
title('Bump sensor occupancy grid with 0,1 value');
xlabel('x position (m)');
ylabel('y position (m)');
legend([pgrd ppos pmap(1)],'Grids','True trajectory','Wall');
axis equal
hold off
