function [DH] = DH(ai,alphai, di, thetai)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
DH = R44z(thetai)*T44z(di)*T44x(ai)*R44x(alphai);
end

