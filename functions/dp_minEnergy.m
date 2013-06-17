function statevect = dp_minEnergy(statevect,state,constraint,slope,param,ns)
% [statevect] = DP_MINENERGY(statevect,state,constraint,slope,param)
%
% This function finds the subtotal minimum energy of the possible states
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

%limit the acceleration to the specified max
if abs(state.a) > param.lim.acc
    statevect = reseststruct(statevect,'nan');
    return
end

%based on type, apply correct normal force equations
switch slope.type
    case 'downhill'
        F.n1 = param.m*param.g/param.L*(1/2*cosd(slope.phi) +...
            param.b*sind(slope.phi)) - param.m*param.b/param.L*state.a;
        F.n2 = param.m*param.g/param.L*(1/2*cosd(slope.phi) -...
            param.b*sind(slope.phi)) + param.m*param.b/param.L*state.a;
    case 'level'
        F.n1 = param.m*param.g/2 - param.m*param.b/param.L*state.a;
        F.n2 = param.m*param.g/2 + param.m*param.b/param.L*state.a;
    case 'uphill'
        F.n1 = param.m*param.g/param.L*(1/2*cosd(slope.theta) -...
            param.b*sind(slope.theta)) + param.m*param.b/param.L*state.a;
        F.n2 = param.m*param.g/param.L*(1/2*cosd(slope.theta) +...
            param.b*sind(slope.theta)) - param.m*param.b/param.L*state.a;
end

%forces on EGV from FBD
F.g = param.m*param.g*sind(slope.total(ns.k)); %gravity component along slope
F.w = param.Ca*state.v.avg^2;                   %aerodynamic resistance (drag)
F.a = param.m*state.a;                          %acceleration resistance
%calculate required power based on forces
state.P.req   = state.v.avg/2*(F.g + F.w + F.a);
state.P.front = state.w.front.*state.T.front; %P1<0 generator, P1>=0 motor
state.P.rear  = state.P.req - state.P.front;  %satisfy speed equality constraints
state.T.rear  = state.P.rear./state.w.rear;

%---------------------------------%
%------- apply constraints -------%
%1: torque boundary constraints
constraint.T.torquerange = find(state.T.rear>=param.lim.T2.min & ...
    state.T.rear<=param.lim.T2.max);
  state.T.front = state.T.front(constraint.T.torquerange);
  state.T.rear  = state.T.rear(constraint.T.torquerange);
%2: torque terrain (normal force) constraints
constraint.T.Fn2 = find(state.T.rear <= F.n2*param.mu*param.R_eff);
  state.T.front = state.T.front(constraint.T.Fn2);
  state.T.rear  = state.T.rear(constraint.T.Fn2);
constraint.T.Fn1 = find(state.T.front <= F.n1*param.mu*param.R_eff);
  state.T.front = state.T.front(constraint.T.Fn1);
  state.T.rear  = state.T.rear(constraint.T.Fn1);
%------- apply constraints -------%
%---------------------------------%

%--POPULATE VECTORS WITH POSSIBLE CONTROL VALUES--
%find lengths of constrained torque vectors
a(1) = length(state.T.front);
a(2) = length(state.T.rear);
if a(1)~=a(2)
    %if the torque vectors are not the same size, something went wrong
    error('T1 and T2 have different dimensions')
elseif a(1)==0
    %if no torques fall within specified range, set everything to NANs
    statevect = resetstruct(statevect,'nan');
    return;
end
%get motor efficiencies at specified speed and torque values
for i=1:a(1)
    motorEff.front(i) = getMotorEff(state.v.avg,state.T.front(i),para.R_eff);
    motorEff.rear(i)  = getMotorEff(state.v.avg,state.T.rear(i),para.R_eff);
end
%evaluate efficiency based on the possible torque values
eta.front = zeros(a(1),1);
eta.rear  = zeros(a(2),1);
for i=1:a(1)
    if state.T.front(i)<0
        eta.front(i) = param.eff.brake.front*motorEff.front(i);
        if state.T.rear(i)<0
            state.P.type = 'regenBoth';
            eta.rear(i)  = param.eff.brake.rear*motorEff.rear(i);
        else
            state.P.type = 'regenFront';
            eta.rear(i)  = 1/(param.eff.drive.rear*motorEff.rear(i));
        end
    else
        eta.front(i) = 1/(param.eff.drive.front*motorEff.front(i));
        if state.T.rear(i)<0
            state.P.type = 'regenRear';
            eta.rear(i)  = param.eff.brake.rear*motorEff.rear(i);
        else
            state.P.type = 'regenNone';
            eta.rear(i)  = 1/(param.eff.drive.rear*motorEff.rear(i));
        end
    end
end
%calculate power
Pg1 = state.w.front.*state.T.front;
Pg2 = state.w.rear.*state.T.rear;
state.P.total = Pg1.*eta.front + Pg2.*eta.rear;

%--POPULATE VECTORS EITH POSSIBLE STATE VALUES--
%calculate change in state of charge
SOC.delta = state.P.total*state.dt/(param.E_max*param.V_bat*param.conv.h2s);
%---------------------------------%
%------- apply constraints -------%
%3: state of charge boundary constraint
SOC.currmin = min(matr.SOE(ns.currspd,:,ns.k));
SOC.currmax = max(matr.SOE(ns.corrspd,:,ns.K));
constraint.SOC.range = find(SOC.currmin+SOC.delta <= param.lim.SOE.max |...
                            SOC.currmax+SOC.delta >= param.lim.SOE.min);
%apply constraints to power and torque vectors
state.P.total = state.P.total(constraint.SOC.range);
state.T.front = state.T.front(constraint.SOC.range);
state.T.rear  = state.T.rear(constraint.SOC.range);
%find min value and index
[state.P.min,ind_pmin] = min(state.P.total);
%insert the calculated min into the slot for the next speed
if isempty(state.T.front(ind_pmin)) || isempty(state.T.rear(ind_pmin))
    statevect = resetstruct(statevect,'nan');
else
    statevect.P(ns.nextspd) = state.P.min;
    statevect.T.front(ns.nextspd) = state.T.front(ind_pmin);
    statevect.T.rear(ns.nextspd) = state.T.rear(ind_pmin);
end
%------- apply constraints -------%
%---------------------------------%

%--CALCULATE MIN ENERGY CONSUMPTION OF ALL POSSIBLE STATES FROM THE NEXT
%POINT TO THE DESTINATION--
statevect.E.sub(ns.nextspd) = statevect.P(ns.nextspd)*statevect.t(ns.nextspd);
if k==N
    statevect.E.subtot(ns.nextspd) = statevect.E.sub(ns.nextspd) + param.E_final;
else
    ind_nextv = find(abs(vect.v - state.v/param.conv.kmh2mps) <= 0.001);
    if isempty(ind_nextv)
        statevect.E.subtot(ns.nextspd) = NaN;
    else
        statevect.E.subtot(ns.nextspd) = statevect.E.sub(ns.nextspd)+...
            tbl.E(ind_nextv,ns.k+1);
    end
end














