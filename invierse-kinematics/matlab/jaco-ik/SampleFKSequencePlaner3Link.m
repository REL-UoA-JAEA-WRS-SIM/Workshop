%
% 3-Link Robot Arm Forward Kinematics by DH method
%   Draw a series of robot poses for a given set joint angles  
%   Author: Keitaro Naruse
%   Date: 2020-03-11
%

% Link lengths
L1 = 1;
L2 = 1;
L3 = 1;

% Joint angles
q1 = [0:1:90];
q2 = [0:1:90];
q3 = [0:1:90];

% Draw in 2D, Loop for joint angles
figure(1);
hold on
for i = 1:91
    % Homegeneous transformation matricies for given angles
    % a frame to previous frame
    T01 = DH(0,  0, 0, deg2rad( q1(i)) );
    T12 = DH(0, L1, 0, deg2rad( q2(i)) );
    T23 = DH(0, L2, 0, deg2rad( q3(i)) );
    T34 = DH(0, L3, 0, deg2rad(  0   ) );

    % a frame to base 
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;

    % Points in base frame
    p00 = [0; 0; 0; 1];
    p01 = T01 * [0; 0; 0; 1];
    p02 = T02 * [0; 0; 0; 1];
    p03 = T03 * [0; 0; 0; 1];
    p04 = T04 * [0; 0; 0; 1];
    
    % Plot
    x = [p00(1), p01(1), p02(1), p03(1), p04(1)];
    y = [p00(2), p01(2), p02(2), p03(2), p04(2)];
    z = [p00(3), p01(3), p02(3), p03(3), p04(3)];
    plot(x, y, '-o');
    xlim([-3, 3]);
    ylim([-3, 3]);
end
hold off
xlim([-3 3]);
ylim([-3 3]);
title('3-link robot arm in 2D');
pbaspect([1 1 1]);

% Draw in 3D, Loop for joint angles
figure(2);
view(3);
hold on
for i = 1:91
    % Homegeneous transformation matricies for given angles
    % a frame to previous frame
    T01 = DH(0,  0, 0, deg2rad( q1(i)) );
    T12 = DH(0, L1, 0, deg2rad( q2(i)) );
    T23 = DH(0, L2, 0, deg2rad( q3(i)) );
    T34 = DH(0, L3, 0, deg2rad(  0   ) );

    % a frame to base 
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;

    % Points in base frame
    p00 = [0; 0; 0; 1];
    p01 = T01 * [0; 0; 0; 1];
    p02 = T02 * [0; 0; 0; 1];
    p03 = T03 * [0; 0; 0; 1];
    p04 = T04 * [0; 0; 0; 1];
    
    % Plot
    x = [p00(1), p01(1), p02(1), p03(1), p04(1)];
    y = [p00(2), p01(2), p02(2), p03(2), p04(2)];
    z = [p00(3), p01(3), p02(3), p03(3), p04(3)];
    plot3(x, y, z, '-o');
end
hold off
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
