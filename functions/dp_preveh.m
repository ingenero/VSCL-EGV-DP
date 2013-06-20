function [badIndexVector] = dp_preveh(tbl,vect,pre,ns)
%
%
% This function makes the EGV stay behind a preceding vehicle.
%
%
% -------------------------------------------------------------------------
% ------------------------------- INPUTS ----------------------------------
%
% -------------------------------------------------------------------------
%
%
% -------------------------------------------------------------------------
% ------------------------------ OUTPUTS ----------------------------------
%
% -------------------------------------------------------------------------

badIndexVector = zeros(ns.NumOfSpds,1);
for startingPoint = 1:ns.NumOfSpds
    for kk = ns.k:ns.N
        if kk==ns.k
            currIndex = startingPoint;
        else
            currIndex = nextIndex;
        end
        
        tempOpt.E(kk,1) = tbl.E(currIndex,kk);
        tempOpt.v(kk,1) = tbl.v(currIndex,kk);
        tempOpt.t(kk,1) = tbl.t(currIndex,kk);
        nextIndex = find(tempOpt.v(kk)==vect.v);
    end
    
    time.EGV = sum(tempOpt.t);
    time.pre = sum(pre.t(ns.k:ns.N+1));
    
    if time.EGV>time.pre
        badIndexVector(startingPoint) = 0;
    else
        badIndexVector(startingPoint) = 1;
    end
end