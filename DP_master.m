% DP_MASTER
%
% This is the master file for an energy optimization program that 
% calculates the optimal trajectory of an electric ground vehicle.
% A dynamic programming algorithm is used.
%
% To run the program, enter the desired inputs (below) and hit 'run'.

clear all; close all; clc

%% User Input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------  USER INPUT  ---------------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--ELECTRIC GROUND VEHICLE (egv)--
egv.v.min  = 25;    %[km/h] minimum allowable velocity
egv.v.max  = 33;    %[km/h] maximum allowable velocity
egv.v.step = 1;     %[km/h] difference between discrete velocities
egv.v.v0   = 26;    %[km/h] starting velocity
egv.v.vN   ='free'; %[km/h] ending velocity ('free' if unspecified)
egv.x.step = 30;    %[m]    distance between discrete nodes
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
%       terrain in another (plots are stacked vertically).
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
%add functions folder to the current path
addpath('functions')

%--LENGTHS USED IN DP--
%set number of discrete states
ns.Nq = 30;

%--IMPORT TERRAIN INFO--
%extract terrain info from 'init_terrain' function
[terrain,ns] = init_terrain(filenames,egv,ns);

%--PHYSICAL PARAMETERS--
%extract intialized variables from 'init_param' function
[param,vect,matr,tbl,opt,state,statevect,slope,constraint] ...
    = init_param(egv,ns);
%store the number of possible speeds
ns.NumOfSpds = length(vect.v);


%% Dynamic Programming
%Start at the last node and calculate the SOE for each possible
%combination of current and next speeds.
k = ns.N;
while k >= 1
    %calculate slope of the stage at the current iteration
    [slope,terrain] = dp_slope(k,terrain,slope,egv);
    
    %--LOOP THROUGH ALL CURRENT SPEEDS--
    for IndexCurrSpd = 1:ns.NumOfSpds
        %set the current speed (convert to m/s)
        if k == 1
            state.v.curr = egv.v.v0*param.conv.kmh2mps;
        else
            state.v.curr = vect.v(IndexCurrSpd)*param.conv.kmh2mps;
        end
        %set the state vectors to zeros
        statevect = resetstruct(statevect,'zero');
        
        %--LOOP THROUGH ALL NEXT SPEEDS--
        for IndexNextSpd = 1:ns.NumOfSpds
            %SET THE NEXT SPEED
            if strcmp(egv.v.vN,'free')
                state.v.next = vect.v(IndexNextSpd)*param.conv.kmh2mps;
            else
                if k == ns.N
                    state.v.next = egv.v.vN;
                else
                    state.v.next = vect.v(IndexNextSpd)*param.conv.kmh2mps;
                end
            end
            %DEFINE NEW STATE PARAMETERS
            %average speed between current and next speeds - used to
            %calculate the power consumption
            state.v.avg = (state.v.curr+state.v.next)/2;
            %time elapsed when travelling between current and next nodes
            state.dt    =  egv.x.step/state.v.avg;
            %acceleration from the current to next speed
            state.a     = (state.v.next-state.v.curr)/state.dt;
        end
        
    end
    
    k = k-1;
end











