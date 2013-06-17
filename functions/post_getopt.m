function opt = post_getopt(tbl,vect,egv,param,ns)
% opt = POST_GETOPT(tbl,vect,egv,ns)
%
% This function populates the optimal vectors
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

E_tot = zeros(ns.N,1);
%set boundary conditions
opt.v(1)            = egv.v.v0;
opt.T.front(ns.N+1) = 0;
opt.T.rear(ns.N+1)  = 0;

%find location of total minimum energies
for k=1:ns.N
    if k==1
        %find first index
        [E_tot(k,1),ind_opt] = min(tbl.E(:,k));
    else
        E_tot(k,1) = tbl.E(ind_opt,k);
    end
    %populate optimal vectors based off index
    opt.v(k+1,1)       = tbl.v(ind_opt,k);
    opt.T.front(k+1,1) = tbl.T.front(ind_opt,k);
    opt.T.rear(k+1,1)  = tbl.T.rear(ind_opt,k);
    %find next index
    ind_opt = find(opt.v(k+1) == vect.v);
end

%calculate energy at each stage
for k=1:ns.N
    if k==ns.N
        opt.E(k,1) = E_tot(k)-param.E_final;
    else
        opt.E(k,1) = E_tot(k)-E_tot(k+1);
    end
end

%calculate the state of charge vector
for k=1:ns.N+1
    if k==1
        opt.SOC(k,1) = param.lim.SOE.ini;
    else
        opt.SOC(k,1) = opt.SOC(k-1,1) - opt.E(k-1,1)/...
            (param.E_max*param.V_bat*param.conv.h2s);
    end
end









