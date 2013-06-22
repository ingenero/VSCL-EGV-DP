function [pre] = init_precedingvehicle(pre,egv,terrain,param,ns)
%
%
% This function interprets the preceding vehicle info.
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

%extract needed data from terrain info
dx = terrain.dist(2)-terrain.dist(1);

if length(pre.v_in)~=length(pre.x_in)
    error('The inputs for the preceding vehicle must have the same length.')
elseif pre.x_in(1)<=0
    error('The preceding vehicle must start in front of the EGV.')
end

NumSpec = length(pre.v_in);
ind = zeros(NumSpec,1);
for i=1:NumSpec
    ind(i) = find(terrain.dist >= pre.x_in(i),1,'first');
end

pre.v = nan(ns.N+1,1);
for i=ind(1):ns.N+1
    if terrain.dist(i)<=pre.x_in(NumSpec)
        pre.v(i,1) = interp1(pre.x_in,pre.v_in,terrain.dist(i));
    else
        pre.v(i,1) = pre.v_in(NumSpec);
    end
end

distToFirstNode = (ind(1)-1)*dx-pre.x_in(1); %[m]
vAvgToFirstNode = (pre.v(ind(1))+pre.v_in(1))/2*param.conv.kmh2mps; %[m/s]
timeToFirstNode = distToFirstNode/vAvgToFirstNode; %[s]

pre.t = nan(ns.N+1,1);
for i=ind(1):ns.N+1
    if i==ind(1)
        pre.t(i,1) = timeToFirstNode;
    else
        v_avg = (pre.v(i-1)+pre.v(i))/2*param.conv.kmh2mps;
        pre.t(i,1) = dx/v_avg;
    end
end

pre.extra = pre.x_in(1)/(egv.v.max*param.conv.kmh2mps);









