function [param,vect,matr,tbl,opt,state,statevect,slope,constraint] ...
    = init_param(egv,ns)
% [param,vect,matr,tbl,opt,state,statevect,slope,constraint] = INIT_PARAM(egv,ns)
%
% This function outputs all the paramters necessary to calculate the
% optimal trajectory of and EGV based on energy consumption. Included are
% physical constants concerning the EGV as well as efficiencies and
% important ratios for calculating energy.
%
% Also generated are empty vectors and matrices that will be used in the
% dynamic programming algorithm used to calculate the global optimal
% solution.
%
% -------------------------------------------------------------------------
% ------------------------------- INPUTS ----------------------------------
%   egv = structure which contains user input information about the
%         constraints on the EGV
%     ~.v.min  = [km/h] minimum speed of the EGV
%     ~.v.max  = [km/h] maximum speed of the EGV
%     ~.v.step = [km/h] difference between discrete speed
%
%   ns  = structure which contains lengths of important vectors
%     ~.N  = total number of nodes
%     ~.Nq = number of discrete energy states
% -------------------------------------------------------------------------
%
%
% -------------------------------------------------------------------------
% ------------------------------ OUTPUTS ----------------------------------
%   param = structure which contains efficiencies, physical measurements,
%           limits of the EGV, and conversion factors
%
%   vect  = structure which constians vectors of all possible values of a 
%           given parameter
%
%   matr  = structure which contains matrices used to store energy values
%           calculated during each iteration of the DP algorithm
%
%   tbl   = structure which contains tables used to store the states
%           calculated during the DP algorithm
%
%   opt   = structure which contains vectors representing the solution to
%           the optimal solution from a given initial condition
%
%   state = structure which contains the possible states at the current
%           iteration
%
%   slope = structure which contains information
%
%   statevect = structure which contains minimum of state calculated at
%               each new speed
%
%   constraint = structure which will contain vectors enforcing the given
%                constraints on the different states
% -------------------------------------------------------------------------

%drive/brake efficiencies
param.eff.drive.front = 1;
param.eff.drive.rear  = 0.8;
param.eff.drive.p1    = -7.2888e-008;
param.eff.drive.p2    = 1.8023e-005;
param.eff.drive.p3    = -0.0016099;
param.eff.drive.p4    = 0.057038;
param.eff.drive.p5    = 0.16446;
param.eff.brake.front = 1;
param.eff.brake.rear  = 0.8;
param.eff.brake.p1    = 3.5227e-006;
param.eff.brake.p2    = -0.00061109;
param.eff.brake.p3    = 0.034213;
param.eff.brake.p4    = 0.010455;
%physical characteristics of EGV
param.slip.k   = 100000; %coefficient of slipping friction
param.slip.min = -0.1;
param.slip.max = 0.1;
param.Ca	   = 0.37;   %air drag coefficient
param.mu	   = 0.8;	 %tire-road max coeff. of static friction
param.m		   = 800;	 %[kg] mass of EGV
param.g		   = 9.81;	 %[m/s^2] acceleration of gravity
param.L		   = 1.84;	 %[m] wheelbase
param.b		   = 0.8;	 %[m] CG height
param.R_eff	   = 0.3;	 %[m] effective radius of tires
param.E_max	   = 50;	 %[Ah] Li-Fe battery capacity
param.E_final  = 0;		 %[Ah] desired final energy value
param.V_bat	   = 72;	 %[V] total voltage (assume it is fixed)
%constraints
param.lim.T1.min  = -80; %[N-m] front tires min torque
param.lim.T1.max  = 100; %[N-m] front tires max torque
param.lim.T2.min  = -80; %[N-m] rear tires min torque
param.lim.T2.max  = 100; %[N-m] rear tires max torque
param.lim.SOE.min = 0.5; %min charge of the battery
param.lim.SOE.max = 0.8; %max charge of the battery
param.lim.SOE.ini = 0.7; %initial charge of the battery
param.lim.acc     = 0.1*param.g; %max acceleration rate of EGV
%conversion factors
param.conv.km2m	   = 1000; %convert kilometers to meters
param.conv.h2s	   = 3600; %convert hours to seconds
param.conv.kmh2mps = param.conv.km2m/param.conv.h2s;

%--INITIALIZE PARAMETERS TO CALCULATE--
v2struct(ns); %extract numbers for generating matrices
%lists of all possible state values
vect.v    = (egv.v.min:egv.v.step:egv.v.max);
NumOfSpds = length(vect.v);
vect.SOE  = linspace(param.lim.SOE.min,param.lim.SOE.max,Nq);
%populate matrices used to store energy values
matr.SOE2d = zeros(NumOfSpds, Nq);
matr.SOE   = zeros(NumOfSpds, Nq, N);
for i=1:NumOfSpds
	matr.SOE2d(i,:) = vect.SOE;
end
for i=1:N
	matr.SOE(:,:,i) = matr.SOE2d;
end
%tables used to store all possible states
tbl.v       = zeros(NumOfSpds, N);
tbl.T.front = zeros(NumOfSpds, N);
tbl.T.rear  = zeros(NumOfSpds, N);
tbl.P       = zeros(NumOfSpds, N);
tbl.E       = zeros(NumOfSpds, N);
tbl.t       = zeros(NumOfSpds, N);
%vectors containing states at the optimal trajectory
opt.v	    = zeros(N+1, 1);
opt.T.front	= zeros(N+1, 1);
opt.T.rear  = zeros(N+1, 1);
opt.P       = zeros(N+1, 1);
opt.E       = zeros(N+1, 1);
opt.SOC     = zeros(N+1, 1);
opt.t	    = zeros(N+1, 1);
opt.t_cum   = zeros(N+1, 1);
%vectors with all possible states at current iteration
state.T.front  = (param.lim.T1.min:param.lim.T1.max)';
state.T.rear   = (param.lim.T2.min:param.lim.T2.max)';
state.length   = length(state.T.front);
state.P.front  = zeros(state.length, 1);
state.P.rear   = zeros(state.length, 1);
state.P.total  = zeros(state.length, 1);
state.E.sub	   = zeros(state.length, 1);
state.E.subtot = zeros(state.length, 1);
state.t		   = zeros(state.length, 1);
state.v.curr   = 0;
state.v.next   = 0;
state.v.avg    = 0;
state.dt       = 0;
state.a        = 0;
state.w.front  = 0;
state.w.rear   = 0;
%vectors with minimum of all possible states at each possible speed
statevect.E.sub	   = zeros(NumOfSpds, 1);
statevect.E.subtot = zeros(NumOfSpds, 1);
statevect.T.front  = zeros(NumOfSpds, 1);
statevect.T.rear   = zeros(NumOfSpds, 1);
statevect.P		   = zeros(NumOfSpds, 1);
statevect.t		   = zeros(NumOfSpds, 1);
%vectors with slope information
slope.total = zeros(N, 1);
%vectors used to choose values within constraints
constraint.T.range	  = 0;
constraint.T.frontmax = 0;
constraint.T.rearmax  = 0;
constraint.SOC        = 0;

fprintf('\tt_EGV\tt_pre\t v_EGV\t v_pre\n\t------  ------   ------  ------\n')
