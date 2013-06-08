function terrain = init_terrain(filenames,egv,ns)
% terrain = INIT_TERRAIN(filenames,egv,ns)
%
% This function extracts the terrain information from the specified file
% and truncates it to the specified ending distance
%
%
% -------------------------------------------------------------------------
% ------------------------------- INPUTS ----------------------------------
%   filenames = structure containting location and names of the files
%     ~.folder  = name of folder where the data is stored
%     ~.terrain = name of the file to import
%
%   egv = structure which contains constraints on the EGV
%     ~.x.xN = distance at which to end the calculation
% -------------------------------------------------------------------------
%
%
% -------------------------------------------------------------------------
% ------------------------------ OUTPUTS ----------------------------------
%   terrain = structure containing information about the terrain
%     ~.path = full path of the terrain data file
%     ~.info = all data contained in the terrain data file
%     ~.last = node corresponding to the final specified distance
%     ~.dist = truncated vector containing distance of path
%     ~.alti = truncated vector containing altitude of path
% -------------------------------------------------------------------------

terrain.path = [filenames.folder filenames.terrain]; %path to terrain data
terrain.info = importdata(terrain.path); %import data from specified file
%initialize vectors
terrain.dist = zeros(ns.N+1,1);
terrain.alti = zeros(ns.N+1,1);
%find where the last node should be
if ischar(egv.x.xN)
    if strcmp(egv.x.xN,'last')
        terrain.last = length(terrain.info);
        %populate vectors with distance and altitude
        terrain.dist(ns.N+1) = terrain.info(terrain.last,1);
        terrain.alti(ns.N+1) = terrain.info(terrain.last,2);
    else
        disp('Improper ending position entered.')
        return;
    end
else
    allowableNodes = find(terrain.info(:,1) <= egv.x.xN);
    terrain.last = find(allowableNodes,1,'last') + 1;
    %populate vectors with distance and altitude
    terrain.dist(ns.N+1) = egv.x.xN;
    terrain.alti(ns.N+1) = getTerrain(terrain.info,egv.x.xN);
end