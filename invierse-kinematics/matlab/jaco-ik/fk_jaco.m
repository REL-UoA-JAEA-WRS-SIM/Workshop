function [p, T01, T02, T03, T04, T05, T06] = fk_jaco(q)
%fk_jaco forward kinematics for jaco arm
%   input
%       - Joint angle vector [rad]
%         q = [q1; q2; q3; q4; q5; q6]
%   output
%       - Joint position vectors as array [m]
%         p = [p0, p1, p2, p3, p4, p5, p6]
%           = [x0, x1, x2, x3, x4, x5, x6;
%              y0, y1, y2, y3, y4, y5, y6;
%              z0, z1, z2, z3, z4, z5, z
%       - Homogeneous transformation matrix from each frame to base
%         T01, T02, T03, T04, T05, T06

% Jaco arm mechanical parameters
D1 = 0.2755;
D2 = 0.4100; 
D3 = 0.2073; 
D4 = 0.0743; 
D5 = 0.0743;
D6 = 0.1687; 
e2 = 0.0098;

% Jaco arm altenate mechanical parameters
aa = 11*pi/72;
ca = cos(aa);
sa = sin(aa);
c2a = cos(2*aa);
s2a = sin(2*aa);
d4b = (D3 + sa/s2a*D4);
d5b = (sa/s2a*D4 + sa/s2a*D5);
d6b = (sa/s2a*D5 + D6);

% Homegeneous transformation matrix from current to previous
% ClassicalDH table of Jaco arm
T01 = DH(     0,  0,  D1,  q(1) );
T12 = DH( -pi/2,  0,   0,  q(2) );
T23 = DH(     0, D2,  e2,  q(3) );
T34 = DH( -pi/2,  0, d4b,  q(4) );
T45 = DH(  2*aa,  0, d5b,  q(5) );
T56 = DH(  2*aa,  0, d6b,  q(6) );

% Homegeneous transformation matrix from current to base 
% each frame to base 
T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;
T06 = T05 * T56;

% Points in base frame
% Base position in the base frame
p00 = [0; 0; 0; 1];
% Frame-1 = Shoulder position in the base frame
p01 = T01 * [0; 0; 0; 1];
% Frame-2 = Shoulder position in the base frame
p02 = T02 * [0; 0; 0; 1];
% Frame-3 = Elbow position in the base frame
p03 = T03 * [0; 0; 0; 1];
% Frame-4 = Wrist1 position in the base frame
p04 = T04 * [0; 0; 0; 1];
% Frame-5 = Wrist2 position in the base frame
p05 = T05 * [0; 0; 0; 1];
% Frame-6 = Hand position in the base frame
p06 = T06 * [0; 0; 0; 1];

% Convert from 4D homegeneous transformation vector 
% to 3D regular position vector
p = [p00(1:3), p01(1:3), p02(1:3), p03(1:3), p04(1:3), p05(1:3), p06(1:3)];
end

