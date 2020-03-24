%
% Jaco Arm Forward Kinematics by Classical DH method
%   Author: Keitaro Naruse
%   Date: 2020-03-12
%

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

% Set joint angles
% Angle set 1: angles in figure 2
q1_Jaco = 270;
q2_Jaco = 180;
q3_Jaco = 180;
q4_Jaco = 0;
q5_Jaco = 0;
q6_Jaco = 0;

% Angle set 2: angles in figure 3
q1_Jaco = 180;
q2_Jaco = 270;
q3_Jaco =  90;
q4_Jaco = 180;
q5_Jaco = 180;
q6_Jaco = 350;

q1_DH = deg2rad(-q1_Jaco );
q2_DH = deg2rad( q2_Jaco - 90 );
q3_DH = deg2rad( q3_Jaco + 90 );
q4_DH = deg2rad( q4_Jaco );
q5_DH = deg2rad( q5_Jaco - 180 );
q6_DH = deg2rad( q6_Jaco + (180 - 80) );

% Homegeneous transformation matrix from current to previous
% ClassicalDH table of Jaco arm
T01 = ClassicalDH(  pi/2,  0,   D1,  q1_DH );
T12 = ClassicalDH(    pi, D2,    0,  q2_DH );
T23 = ClassicalDH(  pi/2,  0,  -e2,  q3_DH );
T34 = ClassicalDH(  2*aa,  0, -d4b,  q4_DH );
T45 = ClassicalDH(  2*aa,  0, -d5b,  q5_DH );
T56 = ClassicalDH(    pi,  0, -d6b,  q6_DH );

% Homegeneous transformation matrix from current to base 
% a frame to base 
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

% Draw a pose of robot arm in 3D
figure(1);
hold on;
view(3);
% 3D plot for robot pose
plot3(x, y, z, 'k-o');

% Draw base frame axis 
o = [0; 0; 0; 1];
xAxis = [0.1;   0;   0; 1];
yAxis = [  0; 0.1;   0; 1];
zAxis = [  0;   0; 0.1; 1];
plot3([o(1),xAxis(1)], [o(2),xAxis(2)], [o(3),xAxis(3)], 'r-');
plot3([o(1),yAxis(1)], [o(2),yAxis(2)], [o(3),yAxis(3)], 'g-');
plot3([o(1),zAxis(1)], [o(2),zAxis(2)], [o(3),zAxis(3)], 'b-');

% Draw shoulder1 frame axis 
T = T01;
o = T*[0; 0; 0; 1];
xAxis = T*[0.1;   0;   0; 1];
yAxis = T*[  0; 0.1;   0; 1];
zAxis = T*[  0;   0; 0.1; 1];
plot3([o(1),xAxis(1)], [o(2),xAxis(2)], [o(3),xAxis(3)], 'r-');
plot3([o(1),yAxis(1)], [o(2),yAxis(2)], [o(3),yAxis(3)], 'g-');
plot3([o(1),zAxis(1)], [o(2),zAxis(2)], [o(3),zAxis(3)], 'b-');

% Draw shoulder2 frame axis 
T = T02;
o = T*[0; 0; 0; 1];
xAxis = T*[0.1;   0;   0; 1];
yAxis = T*[  0; 0.1;   0; 1];
zAxis = T*[  0;   0; 0.1; 1];
plot3([o(1),xAxis(1)], [o(2),xAxis(2)], [o(3),xAxis(3)], 'r-');
plot3([o(1),yAxis(1)], [o(2),yAxis(2)], [o(3),yAxis(3)], 'g-');
plot3([o(1),zAxis(1)], [o(2),zAxis(2)], [o(3),zAxis(3)], 'b-');

% Draw elbow frame axis 
T = T03;
o = T*[0; 0; 0; 1];
xAxis = T*[0.1;   0;   0; 1];
yAxis = T*[  0; 0.1;   0; 1];
zAxis = T*[  0;   0; 0.1; 1];
plot3([o(1),xAxis(1)], [o(2),xAxis(2)], [o(3),xAxis(3)], 'r-');
plot3([o(1),yAxis(1)], [o(2),yAxis(2)], [o(3),yAxis(3)], 'g-');
plot3([o(1),zAxis(1)], [o(2),zAxis(2)], [o(3),zAxis(3)], 'b-');

% Draw wrist1 frame axis 
T = T04;
o = T*[0; 0; 0; 1];
xAxis = T*[0.1;   0;   0; 1];
yAxis = T*[  0; 0.1;   0; 1];
zAxis = T*[  0;   0; 0.1; 1];
plot3([o(1),xAxis(1)], [o(2),xAxis(2)], [o(3),xAxis(3)], 'r-');
plot3([o(1),yAxis(1)], [o(2),yAxis(2)], [o(3),yAxis(3)], 'g-');
plot3([o(1),zAxis(1)], [o(2),zAxis(2)], [o(3),zAxis(3)], 'b-');

% Draw wrist2 frame axis 
T = T05;
o = T*[0; 0; 0; 1];
xAxis = T*[0.1;   0;   0; 1];
yAxis = T*[  0; 0.1;   0; 1];
zAxis = T*[  0;   0; 0.1; 1];
plot3([o(1),xAxis(1)], [o(2),xAxis(2)], [o(3),xAxis(3)], 'r-');
plot3([o(1),yAxis(1)], [o(2),yAxis(2)], [o(3),yAxis(3)], 'g-');
plot3([o(1),zAxis(1)], [o(2),zAxis(2)], [o(3),zAxis(3)], 'b-');

% Draw hand frame axis 
T = T06;
o = T*[0; 0; 0; 1];
xAxis = T*[0.1;   0;   0; 1];
yAxis = T*[  0; 0.1;   0; 1];
zAxis = T*[  0;   0; 0.1; 1];
plot3([o(1),xAxis(1)], [o(2),xAxis(2)], [o(3),xAxis(3)], 'r-');
plot3([o(1),yAxis(1)], [o(2),yAxis(2)], [o(3),yAxis(3)], 'g-');
plot3([o(1),zAxis(1)], [o(2),zAxis(2)], [o(3),zAxis(3)], 'b-');

% Plot format
xlim([-0.5, 1.5]);
ylim([-0.5, 1.5]);
zlim([-0.5, 1.5]);
title('Jaco arm pose in 3D');
pbaspect([1 1 1]);
hold off;


% Copyright (c) <2020>, <Keitaro Naruse>
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met: 
% 
% 1. Redistributions of source code must retain the above copyright notice,
%    this list of conditions and the following disclaimer. 
% 2. Redistributions in binary form must reproduce the above copyright notice,
%    this list of conditions and the following disclaimer in the documentation
%    and/or other materials provided with the distribution.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
% ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
% 
% The views and conclusions contained in the software and documentation are those
% of the authors and should not be interpreted as representing official policies, 
% either expressed or implied, of the FreeBSD Project.
