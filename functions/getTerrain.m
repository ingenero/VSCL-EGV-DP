function DEM = getTerrain(A,x)
% DEM = getTerrain(A,x)
% This function interpolates the terrain info faster than interp1
%
% INPUTS:
%   A = total terrain information
%     column1: lateral distance
%     column2: altitude (height)
%   x = lateral distance at which to find the altitude
%
% OUTPUTS:
%   DEM = altitude at specified distance

% plot(A(:,1),A(:,2),'LineWidth',3); %axis equal;
% ylabel('Altitude (m)','Interpreter','none','FontWeight','bold','FontSize',20);
% xlabel('Distance (m)','Interpreter','none','FontWeight','bold','FontSize',20);

f1 = find(x == A(:,1), 1);
if isempty(f1);
    f2 = find(x > A(:,1));
    if isempty(f2);
        disp('the input coordinate is wrong!');
    else
        pre_index = f2(length(f2));
        f3 = find(x < A(:,1));
        if isempty(f3);
            disp('the input coordinate is wrong!')
        else
            next_index = f3(1);
        end
    end

    a = double((A(next_index, 2) - A(pre_index, 2))/...
        (A(next_index, 1) - A(pre_index, 1)));
    b = double(A(pre_index, 2) - a*A(pre_index, 1));

    DEM = a*x + b;
else
    DEM = A(f1, 2);      
end


end