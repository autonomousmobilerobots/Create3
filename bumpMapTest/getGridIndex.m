function [i,j] = getGridIndex(x,y,n,m,xrng,yrng)
% xrng: [xmin;xmax]
% yrng: [ymin;ymax]
% n   : Grid num in X
% m   : Grid num in Y
% i   : id of grid in X [1~n]
% j   : id of grid in Y [1~m]
[gx,gy]=getGridPosition(1:n,1:m,n,m,xrng,yrng);
agxx = abs(gx-x)';
agyy = abs(gy-y)';
[vi,i]=min(agxx);
[vj,j]=min(agyy);
end