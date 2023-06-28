function [x,y]=getGridPosition(i,j,n,m,xrng,yrng)
% xrng: [xmin;xmax]
% yrng: [ymin;ymax]
% n   : Grid num in X
% m   : Grid num in Y
% i   : id of grid in X [1~n]
% j   : id of grid in Y [1~m]
Dx = xrng(2)-xrng(1);
dx = Dx/n;
Dy = yrng(2)-yrng(1);
dy = Dy/m;
x = xrng(1)+(i-0.5)*dx;
y = yrng(1)+(j-0.5)*dy;
end