function [speedLimit] = dp_preveh(tbl,vect,pre,egv,ns)
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
        if kk==1
            [~,currIndex] = min(tbl.E(:,kk));
        elseif kk==ns.k
            currIndex = startingPoint;
        else
            currIndex = nextIndex;
        end
        
        tempOpt.E(kk,1) = tbl.E(currIndex,kk);
        tempOpt.v(kk+1,1) = tbl.v(currIndex,kk);
        tempOpt.t(kk+1,1) = tbl.t(currIndex,kk);
        nextIndex = find(tempOpt.v(kk+1)==vect.v);
    end
    
    time.EGV = sum(tempOpt.t);
    time.pre = sum(pre.t(ns.k+1:ns.N+1))-pre.t_extra;
    
    if time.EGV < time.pre
        badIndexVector(startingPoint) = 1;
%         fprintf('Time from node %g to the end:\nEGV:\t%3.2f s\npre:\t%3.2f s\n\n',ns.k,time.EGV,time.pre)
%         fprintf('v_EGV = %3.1f km/h\nv_pre = %3.1f km/h\n\n',tempOpt.v(ns.k+1,1),pre.v(ns.k+1,1))
        
%         fprintf('%g:\t%3.1fs\t%3.1fs\t%3.0fkm/h\t%3.0fkm/s\n',...
%             ns.k,time.EGV,time.pre,tempOpt.v(ns.k+1,1),pre.v(ns.k+1,1))
    else
        badIndexVector(startingPoint) = 0;
    end
    
%     fprintf('%g:\t%3.1fs\t%3.1fs\t%3.0fkm/h\t%3.0fkm/s\n',...
%             ns.k,time.EGV,time.pre,tempOpt.v(ns.k+1,1),pre.v(ns.k+1,1))
        
end

badSpeedLocation = badIndexVector==1;
badSpeeds = tbl.v(badSpeedLocation,ns.k);
speedLimit = min(badSpeeds);

% if speedLimit == egv.v.min
%     error('The preceding vehicle constraint is too restrictive.')
% end

