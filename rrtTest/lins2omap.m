function [omap] = lins2omap(lines,xrng,yrng)
% lines: [k*2n]
% omap.polys     : [1*k] poly
% omap.k         : number of polys in omap
% omap.xrng      : [xmin xmax]
% omap.yrng      : [ymin ymax]
% omap.walls     : [4*w]
% omap.wallid    : [2*w]
% omap.vs        : [2*v]
% omap.ids       : [1*v]
omap.xrng = xrng;
omap.yrng = yrng;
omap.k = size(lines,1);
omap.polys = [];
omap.walls = [];
omap.wallid = [];
omap.vs = [];
omap.ids = [];
offId = 0;
for i=1:omap.k
    polyi = line2poly(lines(i,:));
    omap.vs    = [omap.vs polyi.vs];
    omap.walls = [omap.walls polyi.es];
    omap.wallid = [omap.wallid (polyi.eid+[offId;offId])];
    polyi.eid = (polyi.eid+[offId;offId]);
    omap.ids = [omap.ids (polyi.vid+offId)];
    polyi.vid = (polyi.vid+offId);
    offId = polyi.vid(end);
    omap.polys = [omap.polys polyi];
end
% add walls and vs of boundaries
wallSplit = 5;
square = [omap.xrng(1) omap.xrng(2) omap.xrng(2) omap.xrng(1);...
       omap.yrng(1) omap.yrng(1) omap.yrng(2) omap.yrng(2)];
bvs = [];
for i=1:4
    ni = i+1;
    if ni>4
        ni=1;
    end
    for j=1:wallSplit
        inter = square(:,i)+(square(:,ni)-square(:,i))*(j-1)/wallSplit;
        bvs = [bvs inter];
        omap.ids = [omap.ids omap.ids(end)+1];
        omap.wallid = [omap.wallid [omap.ids(end);omap.ids(end)+1]];
    end
end
omap.vs    = [omap.vs bvs];
for i=1:length(bvs)
    ni = i+1;
    if ni>length(bvs)
        ni=1;
    end
    omap.walls = [omap.walls [bvs(:,i);bvs(:,ni)]];
end
omap.wallid(2,end) = omap.ids(end)-length(bvs)+1;
% assume the boundary is built in ccw
lv = size(omap.vs, 2);
omap.bondIds = (lv-length(bvs)+1):lv;
end

function [poly] = line2poly(line)
% line: [1*2n0] [v1x v1y v2x v2y ... 0 0]
% poly.n: number of vertices in the obstacle
% poly.vs: [2*n] list of vertices in the obstacle
% poly.es: [4*n] lists of edges where es(:,i) is [x1 y1 x2 y2]'
% poly.vid: [1*n] lists of ids from 1 to n
% poly.eid: [1*n] lists of id pairs representing walls
line_no0 = [];
for i=1:length(line)
    % if
    line_no0 = [line_no0 line(i)];
end

n = length(line_no0)/2;
poly.n = n;
poly.vs = zeros(2,n);
poly.es = zeros(4,n);
poly.vid = zeros(1,n);
poly.eid = zeros(2,n);
poly.vs(:,1) = line_no0(1:2)';
poly.es(:,1) = [line_no0((end-1):end)';poly.vs(:,1)];
poly.vid(1) = 1;
poly.eid(:,1) = [n;1];
for i=2:n
    poly.vs(:,i) = line_no0((i*2-1):i*2)';
    poly.es(:,i) = [poly.vs(:,i-1);poly.vs(:,i)];
    poly.vid(i) = i;
    poly.eid(:,i) = [i-1;i];
end
end
