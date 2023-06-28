function pid = plotOccupancyGrid(p,logOdds,occThres)
% occThres = [free/nomi;nomi/occu]
[xs,ys] = getGridPosition(1:p.n,1:p.m,p.n,p.m,p.mapRngX,p.mapRngY);
Dx = p.mapRngX(2)-p.mapRngX(1);
dx = Dx/p.n/2;
Dy = p.mapRngY(2)-p.mapRngY(1);
dy = Dy/p.m/2;
lxs = length(xs);
lys = length(ys);
polyxs = zeros(4,lxs*lys);
polyys = zeros(4,lxs*lys);
polycs = zeros(lxs*lys,1);
polynm = [-dx dy; dx dy; dx -dy; -dx -dy];
for i=1:lxs
    for j=1:lys
        cent = [xs(i) ys(j)];
        poly = cent+polynm;
        polyxs(:,(i-1)*lys+(j)) = poly(:,1);
        polyys(:,(i-1)*lys+(j)) = poly(:,2);
        polycs((i-1)*lys+(j))   = logOdds(i,j);
    end
end
% classify
h = occThres(2);
l = occThres(1);
polycs(polycs>h)=1;
%polycs((polycs>l)&(polycs<h))=0.5;
polycs(polycs<l)=0;
pid = patch(polyxs,polyys,polycs);
colormap(flipud(gray));
end