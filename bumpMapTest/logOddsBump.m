function gridMap = logOddsBump(oldMap, x, z, p)
% p.n: num of cells in X dimension
% p.m: num of cells in Y dimension
% p.mapRngX: map range in X [xmin;xmax]
% p.mapRngY: map range in Y [ymin;ymax]
% p.bumpPose: [left,midd,rght] bump sensor pose in Body frame
%             [3*k], k is the bumped number [x1;y1;th1 x2;y2;th2 ...]
% p.sigma: [left,midd,rght] bump sensor data uncertainties
%          [2*k], k is the bumped number [sigx1;sigy1 sigx2;sigy2 ...]
% p.robCont: Robot contour points in body frame [2*nC]
% p.l0: prior occupancy probability of grid cells
% z: bump sensor 0 or 1 measurement [left;midd;rght]
% x: robot pose in the inertial frame [x;y;th]
% oldMap : previous grid map [n*m]
% gridMap:  updated grid map [n*m]
%
% --------<Initialize pOcc>-----------
pOcc = ones(p.n,p.m)*0.5;
% --------<Check if bumped>-----------
% [~, k] = size(p.bumpPose);
sensed = z==1;
if sum(z)>0
% --------<Calculate bump sensor position in Inertial frame>-----------
    numBump = sum(z);
    bspi = zeros(3,numBump);
    bspi(1:2,:) = robot2global(x',p.bumpPose(1:2,sensed)')'; % [2*k]
    bspi(  3,:) = x(3)+p.bumpPose(3,sensed);
% --------<Find grids where bspi is in>-----------
    [id_bspi_i,id_bspi_j] = getGridIndex(bspi(1,:)',bspi(2,:)',...
                            p.n,p.m,p.mapRngX,p.mapRngY);
    [fid_bspi_i,fid_bspi_j] = fillGrids(id_bspi_i,id_bspi_j);
    pOcc = updatePOcc(pOcc,fid_bspi_i,fid_bspi_j,0.8);
% --------<Calculate 8 1-sigma points>-----------
    pi = 3.14159; nSig = 8;
    dsigangs = 2*pi/nSig;
    sig_angs = -pi:(dsigangs):(pi-dsigangs);
%     bspi_1sig = zeros(2,nSig*numBump);
    for i=1:numBump
        a = p.sigma(1,i); b = p.sigma(2,i);
        sigs = [a*cos(sig_angs);b*sin(sig_angs)];
        sigsi = robot2global(bspi(:,i)',sigs')';
%         bspi_1sig(:,(nSig*i-(nSig-1)):nSig*i) = robot2global(bspi(:,i)',sigs')';
% --------<Find grids where bspi 1-sig is in>-----------
        [id_bs1s_i,id_bs1s_j] = getGridIndex(sigsi(1,:)',sigsi(2,:)',...
                                p.n,p.m,p.mapRngX,p.mapRngY);
        [fid_bs1s_i,fid_bs1s_j] = fillGrids(id_bs1s_i,id_bs1s_j);
        pOcc = updatePOcc(pOcc,fid_bs1s_i,fid_bs1s_j,0.6);
    end
end
% --------<Calculate Robot Contour in Inertial Frame>-----------
robConI = robot2global(x',p.robCont')';
% --------<Calculate grids where Robot Contour is in>-----------
[id_rbct_i,id_rbct_j] = getGridIndex(robConI(1,:)',robConI(2,:)',...
                        p.n,p.m,p.mapRngX,p.mapRngY);
[fid_rbct_i,fid_rbct_j] = fillGrids(id_rbct_i,id_rbct_j);
% robotOccu = id_rbct
pOcc = updatePOcc(pOcc,fid_rbct_i,fid_rbct_j,0.25);
% --------<Update log odds>-----------
l0 = 0;
% prob = pOcc
gridMap = oldMap + log10(pOcc./(1-pOcc)) - p.l0;
end

function pUpdated = updatePOcc(pOld,is,js,v)
l = length(is);
for i=1:l
    pOld(is(i),js(i)) = 0.5*pOld(is(i),js(i))+0.5*v; % max(pCur(is(i),js(i)), pOld(is(i),js(i))+v);
end
pUpdated = pOld;
end

function [fis,fjs] = fillGridsLag(is,js)
fis = [];
fjs = [];
for j=min(js):max(js)
    iInThisJ = is(js==j);
    tis = [];
    if isempty(iInThisJ)
        nj = min(js(js>j));
        lj = max(js(js<j));
        iInNextJ = is(js==nj);
        iInLastJ = is(js==lj);
        tis_min = round((min(iInNextJ)*(j-lj)+min(iInLastJ)*(nj-j))/(nj-lj));
        tis_max = round((max(iInNextJ)*(j-lj)+max(iInLastJ)*(nj-j))/(nj-lj));
        tis = tis_min:tis_max;
    else
        tis = min(iInThisJ):max(iInThisJ);
    end
    tjs = j*ones(1,length(tis));
    fis = [fis tis];
    fjs = [fjs tjs];
end
end

function [fis,fjs] = fillGrids(is,js)
% Link ij pairs
lis = []; ljs = []; pur = 1e-3;
is=[is is(1)];
js=[js js(1)]; % pad
for i=1:(length(is)-1)
    [sj, ids] = min([js(i),js(i+1)]);
    [bj, idb] = max([js(i),js(i+1)]);
    lj = sj+1; li = is(i+ids-1);
    uj = bj-1; ui = is(i+idb-1);
    for j=lj:uj
        wl = (uj-j+pur)/(uj-lj+2*pur);
        wr =   (j-lj+pur)/(uj-lj+2*pur);
        tis = round((li*wl+ui*wr)/(wl+wr));
        lis = [lis tis];
        ljs = [ljs   j];
    end
end
% linkis = lis
% linkjs = ljs
% fill up i along j
eis = [is lis]; ejs = [js ljs];
fis = []; fjs = [];
for j=min(ejs):max(ejs)
    iInThisJ = eis(ejs==j);
    tis = min(iInThisJ):max(iInThisJ);
    tjs = j*ones(1,length(tis));
    fis = [fis tis];
    fjs = [fjs tjs];
end
end
