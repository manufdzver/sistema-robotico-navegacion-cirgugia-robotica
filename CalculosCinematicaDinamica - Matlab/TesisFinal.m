clear
clc
syms l1 q1 q2 l2 alpha lambda x y Z pi pdx pdy pax pay Dm m1 m2 m3

syms d1 a2 a3 q1 q2 q3 x y Z alpha lambda xa ya xd yd ai q1punto q2punto q3punto
%d1=127 mm
%a2=129
%a3=133

%DH 
A1 = DH(0, pi/2, d1, q1);
A1 = subs(A1, cos(pi/2), 0);
A1 = subs(A1, sin(pi/2), 1)

A2 = DH(a2, 0, 0, q2);
A2 = subs(A2, cos(pi/2), 0);
A2 = subs(A2, sin(pi/2), 1)
A2 = [1 0 0 a2*cos(q2);
    0 1 0 a2*sin(q2);
    0 0 1 0
    0 0 0 1]

A3 = DH(a3, 0, 0, q3);
A3 = subs(A3, cos(pi/2), 0);
A3 = subs(A3, sin(pi/2), 1)
A3 = simplify(A3)

A02 = (A1*A2)

T03 = simplify(A1*A2*A3)


% --- Define Center of Mass Locations (in their respective link frames) ---
% You need to define these based on your robot's geometry and mass distribution.
% These are just examples - REPLACE WITH YOUR ACTUAL VALUES.
com_offset1 = [0; 0; 0];      % Assuming COM of link 1 is at its origin
com_offset2 = [a2/2; 0; 0];    % Assuming COM of link 2 is halfway along its length
com_offset3 = [a3/2; 0; 0];    % Assuming COM of link 3 is halfway along its length

% --- Calculate Potential Energy ---
potential_energy = 0;
g_vector = [0; 0; -9.81]; % Assuming gravity acts in the negative z-direction

% Link 1 Potential Energy
T0_com1 = A1 * [eye(3) com_offset1; 0 0 0 1];
p_com1_0 = T0_com1(1:3, 4);
potential_energy = potential_energy + m1 * g_vector' * p_com1_0;

% Link 2 Potential Energy
T02_com2_local = [eye(3) com_offset2; 0 0 0 1]; % Transformation from frame 2 origin to link 2 COM in frame 2
T0_com2 = A02 * T02_com2_local;
p_com2_0 = T0_com2(1:3, 4);
potential_energy = potential_energy + m2 * g_vector' * p_com2_0;

% Link 3 Potential Energy
T03_com3_local = [eye(3) com_offset3; 0 0 0 1]; % Transformation from frame 3 origin to link 3 COM in frame 3
T0_com3 = T03 * T03_com3_local;
p_com3_0 = T0_com3(1:3, 4);
potential_energy = potential_energy + m3 * g_vector' * p_com3_0

% Display the total potential energy
disp('Total Gravitational Potential Energy:');
disp(simplify(potential_energy));



T03T = subs(T03, q1, 0);
T03T = subs(T03, q2, 0);
T03T = subs(T03, q3, 0);
T03T = subs(T03, cos(pi/2), 0);
T03T = subs(T03, sin(pi/2), 1);
T03T = subs(T03, cos(pi), -1);
T03T = subs(T03, sin(pi), 0)

DH = {A1, A2, A3, T03};
revOpris = [1,1,1];
JG = simplify(jacobiano(DH, revOpris))

J11 = JG(1:3,1:3)
determinante = simplify(det(J11))

Rc = T03(1:3,1:3)
Rc2 = simplify(T03(1:3,1:3)*R33z(pi))

Jimg = [(alpha*lambda)/Z, 0, x/Z, -((x*y)/(alpha*lambda)),(alpha^2*lambda^2+x^2)/(alpha*lambda), y;
        0, (alpha*lambda)/Z, y/Z, -((alpha^2*lambda^2+y^2)/(alpha*lambda)), (x*y)/(alpha*lambda), -x]

Rct = [transpose(Rc), zeros(3);
        zeros(3), transpose(Rc)]

RJ= simplify(expand(Rct*JG))

JimRJ= simplify(expand(Jimg*Rct*JG), "steps",300)
JimRJ = subs(JimRJ, cos(pi), -1);
JimRJ = subs(JimRJ, sin(pi), 0);
JimRJ = subs(JimRJ, sin(pi+q2+q3), -sin(q2+q3));
JimRJ = subs(JimRJ, cos(pi+q2+q3), -cos(q2+q3));
JimRJ = subs(JimRJ, cos(pi), -1);
JimRJ = subs(JimRJ, sin(pi), 0);
JimRJ = subs(JimRJ, sin(pi+q3), -sin(q3));
JimRJ = subs(JimRJ, cos(pi+q3), -cos(q3))
% meters
lambdaR = 0.0037 
% pixels/m
alphaR = 706.71/Z 

JimRJt=transpose(JimRJ)

s = size(JimRJ)

syms kp11 kp22 kv11 kv22 kv33
%Kp= [0.0002, 0;
%    0, 0.0006]
Kp= [kp11, 0;
    0, kp22]

%Kv= [0.10, 0, 0;
%    0, 0.22, 0;
%    0, 0, 0.10]

Kv= [kv11, 0, 0;
    0, kv22, 0;
    0, 0, kv33]

E0=[x;
    y]
Ed=[pdx;
    pdy]

Etilde= Ed-E0

qpunto = [q1punto;
    q2punto;
    q3punto]
kvp = Kv*qpunto

g=[0;
    0.0509*cos(q2);
    0.0516*cos(q3)]

%d1=127 mm
%a2=129
%a3=133
tau = transpose(JimRJ)*Kp*Etilde-kvp+g
tau1 = simplify(tau, "steps",300)
tau1 = subs(tau1, alpha, 706.71);
tau1 = subs(tau1, lambda, 0.0037);
tau1 = subs(tau1, d1, 0.127);
tau1 = subs(tau1, a2, 0.129);
tau1 = subs(tau1, a3, 0.133);
tau2 = simplify(tau1, "steps",300)
s = size(tau2)



