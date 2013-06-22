function [tbl,matr,ns] = dp_maketbl(statevect,vect,matr,tbl,egv,param,ns)
% [tbl,matr] = DP_MAKETBL(statevect,vect,matr,egv,param,ns)
%
% This function calculates the min energy consumption of all possible
% states from the current point to the destination
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

[E_min,ind_Emin] = min(statevect.E.subtot);

% if ns.k==1
%     tbl.E(:,ns.k)              = NaN;
%     tbl.v(:,ns.k)              = NaN;
%     tbl.t(:,ns.k)              = NaN;
%     tbl.T.front(:,ns.k)        = NaN;
%     tbl.T.rear(:,ns.k)         = NaN;
% 
%     tbl.E(ind_Emin,ns.k)       = E_min;
%     tbl.v(ind_Emin,ns.k)       = vect.v(ind_Emin);
%     tbl.t(ind_Emin,ns.k)       = statevect.t(ind_Emin);
%     tbl.T.front(ind_Emin,ns.k) = statevect.T.front(ind_Emin);
%     tbl.T.rear(ind_Emin,ns.k)  = statevect.T.rear(ind_Emin);
%     return
% else
    tbl.E(ns.currspd,ns.k)       = E_min;
    tbl.t(ns.currspd,ns.k)       = statevect.t(ind_Emin);
    tbl.T.front(ns.currspd,ns.k) = statevect.T.front(ind_Emin);
    tbl.T.rear(ns.currspd,ns.k)  = statevect.T.rear(ind_Emin);
    switch egv.v.vN
        case 'free'
            tbl.v(ns.currspd,ns.k) = vect.v(ind_Emin);
        otherwise
            if ns.k==ns.N
                tbl.v(ns.currspd,ns.k) = egv.v.vN;
            else
                tbl.v(ns.currspd,ns.k) = vect.v(ind_Emin);
            end
    end
% end

%---------------------------------%
%------- apply constraints -------%
if ns.k==1
    %do nothing
else
    matr.SOE(ns.currspd,:,ns.k-1) = matr.SOE(ns.currspd,:,ns.N) + ...
        tbl.E(ns.currspd,ns.k)/(param.E_max*param.V_bat*param.conv.h2s);
    for i=1:ns.Nq
        if matr.SOE(ns.currspd,i,ns.k-1) < param.lim.SOE.min ||...
                matr.SOE(ns.currspd,i,ns.k-1) > param.lim.SOE.max
            matr.SOE(ns.currspd,i,ns.k-1) = NaN;
        end
    end
end
%------- apply constraints -------%
%---------------------------------%

ns.indexOfMin(ns.currspd) = ind_Emin;
