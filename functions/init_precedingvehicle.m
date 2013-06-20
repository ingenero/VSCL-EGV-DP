function [distToFirstNode] = init_precedingvehicle(pre,egv,terrain,param)
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

%set sample input data
pre.in.v = [27   28   26]; %[km/h] speed of preceding vehicle
pre.in.x = [ 5 1000 1750]; %[m]    place where speed changes

if length(pre.in.v)~=length(pre.in.x)
    error('The inputs for the preceding vehicle must have the same length.')
elseif pre.in.x(1)<=0
    error('The preceding vehicle must start in front of the EGV.')
end

NumSpec = length(pre.in.v);
ind = zeros(NumSpec,1);
for i=1:NumSpec
    ind(i) = find(terrain.dist>=pre.in.x(i),1,'first');
end

distToFirstNode = (ind(1)-1)*egv.x.step-(pre.in.x(1)); %[m]
