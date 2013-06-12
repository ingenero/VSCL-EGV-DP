function [slope,terrain] = dp_slope(k,terrain,slope,egv)
% [slope,terrain] = DP_SLOPE(k,terrain,slope,egv)
%
% This function calculates the slope of the terrain between the current and
% next nodes. The slope is represented with two accute angles because of
% the way the tangent function calculates angles. In the free-body diagram
% of the EGV, however, the convention is for uphill to be accute and
% downhill to be obtuse. The equations for calculating forces on the
% vehicle reflect this convention.
%
%
% -------------------------------------------------------------------------
% ------------------------------- INPUTS ----------------------------------
%   k = current node
%
%   terrain = structure containing information about the terrain
%     ~.dist(k+1) = vector of distance of path (populated from k+1:N)
%     ~.alti(k+1) = vector of altitude of path (populated from k+1:N)
%
%   slope = structure which contains the slope of the terrain
%     ~.total = vector with all angles set to zero
%
%   egv = structure which contains constraints on the EGV
%     ~.x.step = lateral distance between nodes
% -------------------------------------------------------------------------
%
%
% -------------------------------------------------------------------------
% ------------------------------ OUTPUTS ----------------------------------
%   slope = structure which contains information about the slope
%     ~.total = vector with all angles
%     ~.type  = description of the slope ('uphill', 'downhill', or 'level')
%     ~.grad  = number representing the gradient between the current and
%               next nodes
%     ~.theta = [deg] angle of slope if terrain is positive (uphill)
%     ~.phi   = [deg] angle of slope if terrain is negative (downhill)
%
%   terrain = structure containing information about the terrain
%     ~.dist(k) = vector of distance of path (populated from k:N)
%     ~.alti(k) = vector of altitude of path (populated from k:N)
% -------------------------------------------------------------------------

%determine lateral distance of current node
terrain.dist(k) = terrain.dist(k+1) - egv.x.step;
%if calculated position is negative, artificially force it to be zero
if terrain.dist(k) < 0
    terrain.dist(k) = 0;
end
%interpolate terrain data to find altitude at previous segment
terrain.alti(k) = getTerrain(terrain.info,terrain.dist(k));
%determine if slope is uphill or downhill and assign angle accordingly
heightdiff = terrain.alti(k+1)-terrain.alti(k);
if heightdiff > 0
    slope.type  = 'uphill';
    slope.grad  = heightdiff/egv.x.step;
    slope.theta = atand(slope.grad);
    slope.phi   = 0;
elseif heightdiff == 0
    slope.type  = 'level';
    slope.theta = 0;
    slope.phi   = 0;
else
    slope.type = 'downhill';
    slope.grad  = -heightdiff/egv.x.step;
    slope.theta = 0;
    slope.phi   = atand(slope.grad);
end
%store all angles in a single vector
slope.total(k) = slope.theta - slope.phi;
