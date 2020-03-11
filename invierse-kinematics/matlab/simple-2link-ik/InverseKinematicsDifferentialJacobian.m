%
% 2-Link Robot Arm Inverse Kinematics by Symbolic(Differential) Jacobian
%   Author: Keitaro Naruse
%   Date: 2020-03-05
%

% Link parameter
L1 = 1;
L2 = 1;

% Define symbols
syms q1 q2 Fx1(q1) Fy1(q1) Fx2(q1, q2) Fy2(q1, q2) J(q1, q2)

% Forward kinematics function for the second joint
Fx1(q1) = L1 * cos(q1);
Fy1(q1) = L1 * sin(q1);

% Forward kinematics function for the hand
Fx2(q1, q2) = Fx1(q1) + L2 * cos(q1 + q2);
Fy2(q1, q2) = Fy1(q1) + L2 * sin(q1 + q2);

% Find Jacobain
J(q1, q2) = [diff(Fx2(q1, q2), q1), diff(Fx2(q1, q2), q2);...
    diff(Fy2(q1, q2),q1), diff(Fy2(q1, q2),q2)];

% Hand target position pg
pg = [0; 1];

% Iteration number T=100
T = 100;

% Joint angle array
q1 = zeros(1, T); q1(1) = 0.1;
q2 = zeros(1, T); q2(1) = 0.1;

% Position array of the second joint
x1 = zeros(1, T);
y1 = zeros(1, T);
% Position array of the hand
x2 = zeros(1, T);
y2 = zeros(1, T);

% Inverse kinematics solution
x1(1)  = Fx1(q1(1));
y1(1)  = Fy1(q1(1));
x2(1)  = Fx2(q1(1), q2(1));
y2(1)  = Fy2(q1(1), q2(1));
% Iteration
for t = 1:T-1
    % Forward kinematics
    p = [x2(t); y2(t)];
    
    % Inverse KinematicsFind Jacobian
    Delta_q = 0.1 * eval(inv(J(q1(t), q2(t)))) * (pg - p);
    q1(t+1) = q1(t) + Delta_q(1);
    q2(t+1) = q2(t) + Delta_q(2);
    x1(t+1)  = Fx1(q1(t+1));
    y1(t+1)  = Fy1(q1(t+1));
    x2(t+1)  = Fx2(q1(t+1), q2(t+1));
    y2(t+1)  = Fy2(q1(t+1), q2(t+1));
end

% Plot hand trajectory
figure(1);
plot(x2, y2, 'kx-', pg(1), pg(2), 'ko');
xlim([-2, 2]);
ylim([-2, 2]);
title('Hand trajectory by differential Jacobain');
pbaspect([1 1 1]);

% Plot arm pose trajectory
figure(2);
hold on;
for t=1:T
    plot([0, x1(t), x2(t)], [0, y1(t), y2(t)], 'kx-');
end
plot(pg(1), pg(2), 'ko');
title('Pose trajectory by differential Jacobain');
hold off;
xlim([-2, 2]);
ylim([-2, 2]);
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
