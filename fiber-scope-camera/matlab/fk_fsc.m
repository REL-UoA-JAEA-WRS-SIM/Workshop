function [p, T01, T02, T03, T04, T05, T06] = fk_fsc(q)
%fk_fsc forward kinematics for fiber scope camera
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

% Link paramerers for fiber scope camera
L1 = 0.1; 
L2 = 0.1;
L3 = 0.1; 
L4 = 0.1;
L5 = 0.1; 
L6 = 0.1;

% Homegeneous transformation matrix from current to previous
% Modified DH table of Jaco arm
T01 = ModifiedDH(     0, L1,  0, q(1) );
T12 = ModifiedDH(  pi/2, L2,  0, q(2) );
T23 = ModifiedDH(     0, L3,  0, q(3) );
T34 = ModifiedDH(  pi/2, L4,  0, q(4) );
T45 = ModifiedDH(     0, L5,  0, q(5) );
T56 = ModifiedDH(  pi/2, L6,  0, q(6) );

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
