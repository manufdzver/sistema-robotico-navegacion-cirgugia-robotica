function [Rx] = R44x(ang)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
Rx = [1 0 0 0;
    0 cos(ang) -sin(ang) 0;
    0 sin(ang) cos(ang) 0;
    0 0 0 1] ;
end

