function [Ry] = R33y(ang)
%UNTITLED16 Summary of this function goes here
%   Detailed explanation goes here
Ry = [cos(ang) 0 sin(ang);
    0 1 0;
    -sin(ang) 0 cos(ang)];

end

