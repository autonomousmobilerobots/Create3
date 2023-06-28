function [pmap] = plotOmap(omap)
hold on
axis equal
pmap = [];
for i=1:omap.k
    pmapi = plot([omap.polys(i).vs(1,:) omap.polys(i).vs(1,1)],...
        [omap.polys(i).vs(2,:) omap.polys(i).vs(2,1)],'k-','LineWidth',2);
    pmap = [pmap pmapi];
end
bbx = [omap.xrng(1) omap.xrng(2) omap.xrng(2) omap.xrng(1) omap.xrng(1)];
bby = [omap.yrng(1) omap.yrng(1) omap.yrng(2) omap.yrng(2) omap.yrng(1)];
pbb = plot(bbx,bby,'k-','LineWidth',2);
pmap = [pmap pbb];
pad = 1;
xlim(omap.xrng+[-pad pad]);
ylim(omap.yrng+[-pad pad]);
xlabel('x position (m)');
ylabel('y position (m)');
% hold off
end