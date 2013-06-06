function [minval,minindex] = constrainedmin(inputvect,badindex)
[tempmin,tempindex] = min(inputvect);
if ismember(tempindex,badindex)
	inputvect(tempindex) = NaN;
	[minval,minindex] = constrainedmin(inputvect,badindex);
else
	minval   = tempmin;
    minindex = tempindex;
end