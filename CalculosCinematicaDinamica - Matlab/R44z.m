function [Rz] = R44z(ang)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
Rz = [cos(ang) -sin(ang) 0 0;
    sin(ang) cos(ang) 0 0;
    0 0 1 0;
    0 0 0 1];
end

