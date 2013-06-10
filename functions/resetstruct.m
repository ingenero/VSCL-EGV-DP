function [struct] = resetstruct(struct,set_to)
% [struct] = resetstruct(struct,set_to)
%
% This function sets all fields of a structure to zero. Each element
% retains the same size.
%
%
% -------------------------------------------------------------------------
% ------------------------------- INPUTS ----------------------------------
%   struct = structure with nonzero fields
%   set_to = string describing value to set all elements of the structure
%            accepted values: 'zero', 'one', 'nan'
% -------------------------------------------------------------------------
%
%
% -------------------------------------------------------------------------
% ------------------------------ OUTPUTS ----------------------------------
%   struct = structure with all fields set to zero
% -------------------------------------------------------------------------

if strcmp(set_to,'zero') || strcmp(set_to,'one') || strcmp(set_to,'nan')
    %you enetered an acceptable input!
else
    error(['The ''set_to'' variable must be a string. Accepted values'...
        ' are: ''zero'', ''one'', or ''nan''.'])
end

if isstruct(struct)
    names = fieldnames(struct);
else
    in = input(['The input is not a structure. Reset values anyways?'...
        ' (Y/N)\n'],'s');
    if strcmp(in,'Y') || strcmp(in,'y')
        switch set_to
            case 'zero'
                struct = 0;
            case 'one'
                struct = 1;
            case 'nan'
                struct = NaN;
        end
    end
    
    return;
end

for i = 1:length(names)    
    if isstruct(struct.(names{i}))
        struct.(names{i}) = resetstruct(struct.(names{i}),set_to);
    else
        sz = size(struct.(names{i}));
        switch set_to
            case 'zero'
                struct.(names{i}) = zeros(sz);
            case 'one'
                struct.(names{i}) = ones(sz);
            case 'nan'
                struct.(names{i}) = NaN(sz);
        end
    end
end