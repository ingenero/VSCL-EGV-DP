function [minval,minindex] = constrainedmin(inputvect,badindex)
%
%
% This function calculates the minimum while excluding specified values.
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

[tempmin,tempindex] = min(inputvect);
if ismember(tempindex,badindex)
	inputvect(tempindex) = NaN;
	[minval,minindex] = constrainedmin(inputvect,badindex);
else
	minval   = tempmin;
    minindex = tempindex;
end