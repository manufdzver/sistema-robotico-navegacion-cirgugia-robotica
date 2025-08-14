function [JT] = jacobiano(DH,revOpris)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
n = length(revOpris);
c = length(DH)-1;

JT = sym(zeros(6,n));

O0 = [0;0;0];
Z0 = [0;0;1];

if revOpris(1)== 1
    JT(1:6,1) = [cross(Z0, ((DH{c+1}(1:3,4))- O0)) ; Z0];
else
    JT(1:6,1) = [Z0; [0;0;0]];
end

for i=2:1:c
    aux = DH{1};
    for k=2:i-1
        aux = simplify(aux*DH{k});
    end
        
    if revOpris(i)== 1
        JT(1:6,i) = [cross(aux(1:3,3), (DH{c+1}(1:3,4) - aux(1:3,4)));aux(1:3,3)];
    else
        JT(1:6,i) = [aux(1:3,3);[0;0;0]];
    end
        
end

end

