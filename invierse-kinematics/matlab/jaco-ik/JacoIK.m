%
% Jaco Arm Inverse Kinematics by DH method and Numerical Jacobian
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

% Jaco arm another mechanical parameters
aa = 11*pi/72;
ca = cos(aa);
sa = sin(aa);
c2a = cos(2*aa);
s2a = sin(2*aa);
d4b = (D3 + sa/s2a*D4);
d5b = (sa/s2a*D4 + sa/s2a*D5);
d6b = (sa/s2a*D5 + D6);

% Homegeneous transformation matrix from current to previous
% DH table of Jaco arm
T01 = DH( pi/2,  0,   D1,  Q1 );
T12 = DH( pi,   D2,    0,  Q2 );
T23 = DH( pi/2,  0,  -e2,  Q3 );
T34 = DH( 2*aa,  0, -d4b,  Q4 );
T45 = DH( 2*aa,  0, -d5b,  Q5 );
T56 = DH( pi,    0, -d6b,  Q6 );

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
