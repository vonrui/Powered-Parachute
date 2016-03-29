    function [value,isterminal,direction] = event(t,x)
%   Time when (-height) passes through zero in an increasing direction.
%   June 12, 2015
    global tf
    %value=x(6);  % detect positive x(6) = 0
    if t >= tf
        value = 0;
    else 
        value = 1;
    end
    isterminal  =   1; % stop the integration
    direction   =   1; % positive direction