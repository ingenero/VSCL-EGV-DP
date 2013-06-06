% This is the master file for an energy optimization program that calculates the optimal trajectory of an electric ground vehicle.
% A dynamic programming algorithm is used.

% To run the program, enter the desired inputs (below) and hit 'run'.

clear all; close all; clc

%% User Input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------  USER INPUT  ---------------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--ELECTRIC GROUND VEHICLE (egv)--
egv.v.min  = 25; %[km/h] minimum allowable velocity
egv.v.max  = 33; %[km/h] maximum allowable velocity
egv.v.step = 1;  %[km/h] difference between discrete velocities
egv.v.v0   = 26; %[km/h] starting velocity
egv.v.vN   = 26; %[km/h] ending velocity
egv.x.step = 30; %[m]    distance between discrete nodes
%ending position
%   #    = number representing the ending point [m] of the EGV
% 'last' = choose the full road
egv.x.xN   = 'last';

%PRECEDING VEHICLE (pre)
pre.v  = 27; %[km/h] velocity profile
pre.x0 = 5;  %[m]    starting position (in front of EGV)

%--PLOT/VIEWING OPTIONS--
%option for viewing progress (select ONE)
% 'cw'        = view progress in command window (text)
% 'waitbar'   = view progress in popup window with animated status bar
% 'animation' = view progress via custom animation
view.progress = 'cw';
%option for plotting results (can select multiple - must be a cell {})
% NOTE: all options in a row will be plotted in the same figure
% e.g. {'SOC' 'velocity' 'terrain' ; 'torques' 'terrain'} will plot state
%       of charge, velocity and terrain in one figure and torques and
%       terrain in another (stacked vertically).
% 'none'     = creates no plots
% 'SOC'      = plots state of charge
% 'velocity' = plots the velocity profile
% 'torques'  = plots T1 and T2
% 'distance' = plots distance travelled by the EGV
% 'terrain'  = plots the road altitude
view.results.y  = {'none'};
%option for choosing variable to plot results against (select ONE)
% 'time'     = plot results against time
% 'distance' = plot results against lateral distance
view.results.x = 'distance';

%--FILES TO IMPORT--
filenames.folder  = 'inputdata/';      %name of folder where the data and other .m files are located
filenames.terrain = 'terrainInfo.mat'; %data file containing information about the terrain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------  USER INPUT  ---------------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%% Parameter Initialization
%--IMPORT TERRAIN INFO--
terrain.path = [filenames.folder filenames.terrain]; %path to terrain data
terrain.info = importdata(terrain.path); %import data from specified file
%find where the last node should be
if ischar(egv.x.xN)
    if strcmp(egv.x.xN,'last')
        terrain.last = length(terrain.info);
    else
        disp('Improper ending position entered.')
        break;
    end
else
    terrain.last = find(terrain.info(:,1) <= egv.x.xN, 'last');
end
%populate vectors with distance and altitude
terrain.dist = terrain.info(1:terrain.last,1);
terrain.alti = terrain.info(1:terrain.last,2);
%find total number of steps to take in DP
N = round((terrain.dist(terrain.last)+(egv.x.step-1)/2)/egv.x.step);
%set number of discrete states
Nq = 30;

%--PHYSICAL PARAMETERS--
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
param.lim.acc.max = 0.1*param.g; %max acceleration rate of EGV
%conversion factors
param.conv.km2m	   = 1000; %convert kilometers to meters
param.conv.h2s	   = 3600; %convert hours to seconds
param.conv.kmh2mps = param.conv.km2m/param.conv.h2s;

%--INITIALIZE PARAMETERS TO CALCULATE--
%vectors used to choose values within constraints
constraint.T.range	 = 0;
constraint.T.T1max	 = 0;
constraint.T.T2max	 = 0;
constraint.SOC.range = 0;
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
tbl.v  = zeros(NumOfSpds, N);
tbl.T1 = zeros(NumOfSpds, N);
tbl.T2 = zeros(NumOfSpds, N);
tbl.P  = zeros(NumOfSpds, N);
tbl.E  = zeros(NumOfSpds, N);
tbl.t  = zeros(NumOfSpds, N);
%vectors containing states at the optimal trajectory
opt.v	  = zeros(N+1, 1);
opt.T1	  = zeros(N+1, 1);
opt.T2	  = zeros(N+1, 1);
opt.P	  = zeros(N+1, 1);
opt.E	  = zeros(N+1, 1);
opt.t	  = zeros(N+1, 1);
opt.t_cum = zeros(N+1, 1);
%vectors with all possible states at current iteration
state.T1	   = (param.lim.T1.min:param.lim.T1.max)';
state.T2	   = (param.lim.T2.min:param.lim.T2.max)';
state.length   = length(state.T1);
state.P1	   = zeros(state.length, 1);
state.P2	   = zeros(state.length, 1);
state.P_tot	   = zeros(state.length, 1);
state.E.sub	   = zeros(state.length, 1);
state.E.subtot = zeros(state.length, 1);
state.t		   = zeros(state.length, 1);
state.w1	   = 0;
state.w2	   = 0;
%vectors with minimum of all possible states at each possible speed
statevect.E.sub	   = zeros(NumOfSpds, 1);
statevect.E.subtot = zeros(NumOfSpds, 1);
statevect.T1	   = zeros(NumOfSpds, 1);
statevect.T2	   = zeros(NumOfSpds, 1);
statevect.P		   = zeros(NumOfSpds, 1);
statevect.t		   = zeros(NumOfSpds, 1);













