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
egv.v.vN   = 26; %[km/h] ending velocity ('free' if unspecified)
egv.x.step = 30;    %[m]    distance between discrete nodes
%ending position
%   #    = number representing the ending point [m] of the EGV
% 'last' = choose the full road
egv.x.xN   = 'last';

%PRECEDING VEHICLE (pre)
pre.v_in  = [27   28   29]; %[km/h] velocity profile
pre.x_in  = [ 5 1000 1750]; %[m]    position where velocity takes place

%--PLOT/VIEWING OPTIONS--
%option for viewing progress (select ONE)
% 'none'      = view no indication of the progress
% 'simple'    = view text notifications of the starting and ending points
% 'cw'        = view progress in command window (percentage as text)
% 'waitbar'   = view progress in popup window with animated status bar
% 'animation' = view progress via custom animation
view.progress = 'waitbar';
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
filenames.sound   = 'sms_curium.wav';
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

%--CALCULATE PRECEDING VEHICLE BEHAVIOR--
pre = init_precedingvehicle(pre,terrain,param,ns);


%% Dynamic Programming
iteration_num = 0;
ns.badIndex   = zeros(ns.NumOfSpds,1);
%Start at the last node and calculate the SOE for each possible
%combination of current and next speeds.
k = ns.N;
while k >= 1
    ns.k = k;
    iteration_num = iteration_num+1;
    %calculate slope of the stage at the current iteration
    [slope,terrain] = dp_slope(k,terrain,slope,egv);
    
    %--LOOP THROUGH ALL CURRENT SPEEDS--
    for IndexCurrSpd = 1:ns.NumOfSpds
        ns.currspd = IndexCurrSpd;
        %set the current speed (convert to m/s)
        if k==1
            state.v.curr = egv.v.v0*param.conv.kmh2mps;
        else
            state.v.curr = vect.v(IndexCurrSpd)*param.conv.kmh2mps;
        end
        %set the state vectors to zeros
        statevect = resetstruct(statevect,'zero');
        
        %--LOOP THROUGH ALL NEXT SPEEDS--
        for IndexNextSpd = 1:ns.NumOfSpds
            ns.nextspd = IndexNextSpd;
            %SET THE NEXT SPEED
            switch egv.v.vN
                case 'free'
                    state.v.next = vect.v(IndexNextSpd)*param.conv.kmh2mps;
                otherwise
                    if k==ns.N
                        state.v.next = egv.v.vN*param.conv.kmh2mps;
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
            
            %FIND THE STATE WITH THE MINIMUM ENERGY
            [statevect,constraint] = ...
                dp_getenergymin(statevect,state,tbl,matr,vect,constraint,slope,param,ns);
        end
        
        %POPULATE DP TABLES
        [tbl,matr,ns] = dp_maketbl(statevect,vect,matr,tbl,egv,param,ns);
    end
    
    %--APPLY PRECEDING VEHICLE CONSTRAINT--
%     vect.badIndex = dp_preveh(tbl,vect,pre,ns);
%     if vect.badIndex == 0
%         ns.badIndex = 0;
%         iteration_num = iteration_num+1;
%         k = k-1;
%     else
%         badIndexLocation = find(vect.badIndex==1);
%         ns.badIndex = ns.indexOfMin(badIndexLocation);
%         disp(ns.badIndex)
%     end
    
    %--VIEW CURRENT PROGRESS--
    switch view.progress
        case 'none'
            %do nothing
        case 'simple'
            %display notifications when the DP begins and finishes
            if iteration_num == 1
                disp('DP in progress...')
            elseif iteration_num == ns.N
                disp('DP complete!')
            end
        case 'cw'
            %print the percentage complete in the command window
            perccount(iteration_num,ns.N)
        case 'waitbar'
            %show the progress in a popup window with a status bar
            if iteration_num == 1
                progress.h = waitbar(0,'DP in progress... 0%');
            elseif iteration_num == ns.N
                waitbar(1,progress.h,'DP is complete!');
                close(progress.h)
            else
                progress.frac = iteration_num/ns.N;
                progress.str  = ['DP in progress... '...
                    num2str(ceil(100*progress.frac)) '%'];
                waitbar(progress.frac,progress.h,progress.str)
            end
        case 'animation'
            %display progress via a custom animation (NOT WORKING YET)
            disp('Animation not yet implemented. (DP was cancelled)')
            break;
        otherwise
            %notify the user if an error was made
            error(['An improper viewing selection for the DP was made. '...
                'Specify this in the ''view.progress'' parameter.'])
    end
    k = k-1;
end


%% Post Processing
opt = post_getopt(opt,tbl,vect,egv,param,ns);

figure(1);
subplot(311); plot(terrain.dist,opt.SOC); ylabel('SOC')
subplot(312); plot(terrain.dist,opt.v); ylabel('Velocity (km/h)')
subplot(313); plot(terrain.dist,terrain.alti); ylabel('Altitude (m)')
xlabel('Distance (m)')


%% End Notification
[sound.data,sound.freq] = wavread([filenames.folder filenames.sound]);
wvlngth = length(sound.data);
sound.time = linspace(0, wvlngth/sound.freq, wvlngth);

%increase volume and play sound
loudsound = sound.data*1;
sound.obj =  audioplayer(loudsound,sound.freq);
play(sound.obj)

% figure(1); plot(sound.time,sound.data,'b'); 
% title('Sound Effect Waveform');










