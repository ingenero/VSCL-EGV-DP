function [v,SOC] = forwardsim(torques,terrain,egv,param)
% [v,SOC] = FORWARDSIM(torques,terrain,egv,param)
%
% This function calculates the EGV response, in terms of velocity and state
% of charge, to a given torque distribution.
%
%
% -------------------------------------------------------------------------
% ------------------------------- INPUTS ----------------------------------
%   torques = structure which holds the torque output of the in-wheel
%             electric motors at each node
%     ~.T1 = [N-m] torques in the front wheels
%     ~.T2 = [N-m] torques in the rear wheels
%
%   terrain = structure containing information about the terrain
%     ~.path = full path of the terrain data file
%     ~.info = all data contained in the terrain data file
%     ~.last = node corresponding to the final specified distance
%     ~.dist = [m] truncated vector containing distance of path
%     ~.alti = [m] truncated vector containing altitude of path
%
%   egv = structure which contains user input information about the
%         constraints on the EGV
%     ~.v.min  = [km/h] minimum speed of the EGV
%     ~.v.max  = [km/h] maximum speed of the EGV
%     ~.v.step = [km/h] difference between discrete speed
%     ~.v.v0   = [km/h] initial speed of the EGV
%     ~.x.step = [m]    distance between discrete nodes
%
%   param = structure which contains efficiencies, physical measurements,
%           limits of the EGV, and conversion factors
% -------------------------------------------------------------------------
%
%
% -------------------------------------------------------------------------
% ------------------------------ OUTPUTS ----------------------------------
%   v = structure which contains the speed of the EGV at each node based on
%       the specified input torques
%     ~.si  = [m/s] speed profile in SI units
%     ~.sol = [km/h] speed profile in km/h
%
%   SOC = structure which contains the state of charge of the EGV at each
%         node, calculated from the specified input torques
% -------------------------------------------------------------------------

%Initialization
N = terrain.last-1;

slope.total = zeros(N,1);
SOC         = zeros(N+1,1);
SOC(1)      = param.lim.SOE.ini;
v.si        = zeros(N+1,1);
v.si(1)     = egv.v.v0*param.conv.kmh2mps;

syms v_next


%loop through all nodes for forward simulation of EGV
for k = 1:N
    T1 = torques.T1(k);
    T2 = torques.T2(k);
    
    
    %get the slope from the terrain information
    [slope,terrain] = dp_slope(k,terrain,slope,egv);
    
    %calculate parameters related to velocity
    v_avg = (v.si(k)+v_next)/2;
    d_t    = egv.x.step/v_avg;
    v_dot = (v_next-v.si(k))/d_t;
    
    %set the equations for the forces
    F.x1 = T1/param.R_eff;
    F.x2 = T2/param.R_eff;
    F.t = 2*(F.x1+F.x2);
    F.g = param.m*param.g*sind(slope.total(k));
    F.w = param.Ca*v_avg^2;
    F.a = param.m*v_dot;
    
    %solve for the next velocity of the EGV
    soln = solve(F.t == F.g+F.w+F.a,'v_next');
    v.si(k+1) = double(soln(1));
    
    %calculate rotational velcitites
    v.avg = mean(v.si(k:k+1));
    dt = egv.x.step/v.avg;
    w1 = v.avg/param.R_eff;
    w2 = v.avg/param.R_eff;
    
    %calculate motor effeciencies
    motorEff01 = getMotorEff(v.avg,T1,param.R_eff);
    motorEff02 = getMotorEff(v.avg,T2,param.R_eff);
    if T1<0 && T2<0;
        %powertype = 'regenBoth';
        eta01 = param.eff.brake.front*motorEff01; %regenerative brake
        eta02 = param.eff.brake.rear*motorEff02; %regenerative brake
    elseif T1<0 && T2>=0
        %powertype = 'regenFront';
        eta01 = param.eff.brake.front*motorEff01; %regenerative brake
        eta02 = 1/(param.eff.drive.rear*motorEff02); %drive
    elseif T1>=0 && T2<0
        %powertype = 'regenRear';
        eta01 = 1/(param.eff.drive.front*motorEff01); %drive
        eta02 = param.eff.brake.rear*motorEff02; %regenerative brake
    elseif T1>=0 && T2>=0
        %powertype = 'regenNone';
        eta01 = 1/(param.eff.drive.front*motorEff01); %drive
        eta02 = 1/(param.eff.drive.rear*motorEff02); %drive
    else
        disp('Something went wrong with the power calculation!')
        break
    end
    
    %calculate power
    Pg1 = w1.*T1;
    Pg2 = w2.*T2;
    P_tot = Pg1.*eta01 + Pg2.*eta02;

    %POPULATE VECTORS WITH POSSIBLE STATE VALUES (SOE)
    %calculate change in state of charge of the battery
    delta_SOC = P_tot*dt/(param.E_max*param.V_bat*param.conv.h2s);
    
    SOC(k+1) = SOC(k)-delta_SOC;
    
    perccount(k,N)
end

v.sol = v.si/param.conv.kmh2mps;
