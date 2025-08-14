function [Ry] = R44y(ang)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
Ry = [cos(ang) 0 sin(ang) 0;
    0 1 0 0;
    -sin(ang) 0 cos(ang) 0;
    0 0 0 1];

end

