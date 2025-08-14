function [Rx] = R33x(ang)
%UNTITLED15 Summary of this function goes here
%   Detailed explanation goes here
Rx = [1 0 0;
    0 cos(ang) -sin(ang);
    0 sin(ang) cos(ang)];

end

