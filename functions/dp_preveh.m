function [] = dp_preveh(tbl,ns)
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
        nextIndex = find(opt.v(kk)==vect.v);
    end
    
    time.EGV = sum(tempOpt.t(startingPoint:ns.N));
    time.pre = 
    
end