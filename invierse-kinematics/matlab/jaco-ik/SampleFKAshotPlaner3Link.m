%
% 3-Link Robot Arm Forward Kinematics by DH method
%   Draw a robot pose for given joint angles  
%   Author: Keitaro Naruse
%   Date: 2020-03-11
%

% Link lengths
L1 = 1;
L2 = 1;
L3 = 1;

% Joint angles(degree)
q1 = 10;
q2 = 20;
q3 = 30;

% Planar 3-link robot arms
% DH Table
% i Twist_angle Link_length Offset  Joint_angle;
% 1 0           L0          0       q1
% 2 0           L1          0       q2
% 3 0           L2          0       q3
% 4 0           L3          0       0

% Homegeneous transformation matrix from current to previous
T01 = DH(0,  0, 0, deg2rad(q1) );
T12 = DH(0, L1, 0, deg2rad(q2) );
T23 = DH(0, L2, 0, deg2rad(q3) );
T34 = DH(0, L3, 0, deg2rad(0 ) );

% Homegeneous transformation matrix from current to base 
T02 = T01 * T12;
T03 = T01 * T12 * T23;
T04 = T01 * T12 * T23 * T34;

% Points in base frame
% Joint-1 = Base position in the base frame
p00 = [0; 0; 0; 1];
% Joint-2 position in the base frame
p01 = T01 * [0; 0; 0; 1];
% Joint-2 position in the base frame
p02 = T02 * [0; 0; 0; 1];
% Joint-3 position in the base frame
p03 = T03 * [0; 0; 0; 1];
% Hand position in the base frame
p04 = T04 * [0; 0; 0; 1];

% Draw a pose of robot arm in 2D
figure(1);
% Array for x coordinate of joint-1, joint-2, joint-3, hand 
x = [p00(1), p01(1), p02(1), p03(1), p04(1)];
% Array for y coordinate of joint-1, joint-2, joint-3, hand 
y = [p00(2), p01(2), p02(2), p03(2), p04(2)];
% Array for z coordinate of joint-1, joint-2, joint-3, hand 
z = [p00(3), p01(3), p02(3), p03(3), p04(3)];
% 2D plot for x and y
plot(x, y, '-o');
xlim([-3 3]);
ylim([-3 3]);
title('3-link robot arm in 2D');
pbaspect([1 1 1]);

% Draw a pose of robot arm in 3D
figure(2);
% 3D plot for x, y, z
plot3(x, y, z, '-o');
xlim([-3 3]);
ylim([-3 3]);
zlim([-3 3]);
title('3-link robot arm in 3D');
pbaspect([1 1 1]);

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
