function [isInPoly] = checkInsidePoly(boundIds, qp, vs, ids, maxX)
% D. Sunday 2001
% boundIds: [1*nB]
% vs: [2*n]
% ids: [1*n]
% qp: [2*1]
% maxX: [1*1]
wn = 0;
nB = length(boundIds);
for i=1:nB
    nxti = i+1;
    if nxti>nB
        nxti = 1;
    end
    posi = ids==boundIds(i);
    posni = ids==boundIds(nxti);
    [isInt,~,~] = intersectPoint(qp(1),qp(2),maxX,qp(2),...
        vs(1,posi),vs(2,posi),...
        vs(1,posni),vs(2,posni));
    if isInt
        if vs(2,posni)>vs(2,posi)
            wn = wn+1;
        elseif vs(2,posni)<vs(2,posi)
            wn = wn-1;
        else
            wn = wn;
        end
    end
end
if wn == 0
    isInPoly = 0;
else
    isInPoly = 1;
end
end