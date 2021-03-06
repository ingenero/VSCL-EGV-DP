function [] = post_plot(view,filenames,terrain,opt,pre,egv,param,ns)
% This function plots the results of the DP in a way specified by the user
% via the input format.
%
%
%determine number of figures to create
input = view.results.y;
numOfInputs = length(input);
numOfFigs = str2double(input{numOfInputs}(1));

%check to make sure inputs are kosher
for i=1:numOfInputs
    switch view.results.y{i}
        case 'no_warning'
            return
        case 'none'
            disp('Post-processing complete. No data was plotted.')
            return
        otherwise
            possibleplots = {'SOC','velocity','torques','distance','terrain'};
            cellsize  = length(input{i});
            rawname   = input{i}(3:cellsize);
            cmparray  = strcmp(rawname,possibleplots);
            cmpresult = ismember(1,cmparray);
            if cmpresult==0
                error('Improper selection of y-axis in plots.')
            end
    end
end
switch view.results.x
    case 'time'
        xData = opt.t_cum;
        xName = 'Time (s)';
    case 'distance'
        xData = terrain.dist;
        xName = 'Distance (m)';
    otherwise
        error('Improper selection of x-axis in plots.');
end

%sort inputs into different figures
for fignum = 1:numOfFigs
    %clear variables and create a new figure
    clear cmparray cellsize rawname; figure(fignum)
    
    %get data to plot in current figure
    cmparray = strncmp(input,num2str(fignum),1);
    currinput = input(cmparray);
    numofaxes = length(currinput);
    
    %plot data in subplots
    for i=1:numofaxes
        %get the name of the data to plot (without fig identifier)
        cellsize = length(currinput{i});
        rawname  = currinput{i}(3:cellsize);
        
        %create subplot space
        subplot(numofaxes,1,i)
        
        %get the data to plot
        switch rawname
            case 'SOC'
                yData = opt.SOC;
                yName = 'SOC';
            case 'velocity'
                yData = opt.v;
                yName = 'Velocity (km/h)';
            case 'torques'
                yData(:,1) = opt.T.front;
                yData(:,2) = opt.T.rear;
                yName = 'Torque (N-m)';
            case 'distance'
                %integrate to get position of the vehicle
                yData = cumtrapz(opt.t_cum,opt.v*param.conv.kmh2mps);
                yName = 'Position (m)';
            case 'terrain'
                yData = terrain.alti;
                yName = 'Altitude (m)';
            otherwise
                error('Something went wrong creating subplot.')
        end
        
        %plot data
        plot(xData,yData)
        ylabel(yName); xlabel(xName);
        
        datasz = size(yData);
        if datasz(2)>1
            legend('front','rear','Location','Best')
        end
    end    
end

