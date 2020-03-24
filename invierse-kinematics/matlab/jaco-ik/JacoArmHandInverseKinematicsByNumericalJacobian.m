%
% Jaco Arm Hand Inverse Kinematics by Modified DH method and Numerical Jacobian
%   Author: Keitaro Naruse
%   Date: 2020-03-13
%

% Set joint angles
% Angle set 1: angles in figure 2 [deg]
% q1_Jaco = 270;
% q2_Jaco = 180;
% q3_Jaco = 180;
% q4_Jaco = 0;
% q5_Jaco = 0;
% q6_Jaco = 0;

% Angle set 2: angles in figure 3 [deg]
q1_Jaco = 180;
q2_Jaco = 270;
q3_Jaco =  90;
q4_Jaco = 180;
q5_Jaco = 180;
q6_Jaco = 350;

% Convert into standard DH angle [rad]
% It is because Jaco arm introduces a unique angle definition 
q1_DH = deg2rad(-q1_Jaco + 180 );
q2_DH = deg2rad( q2_Jaco - 270 );
q3_DH = deg2rad(-q3_Jaco +  90 );
q4_DH = deg2rad(-q4_Jaco + 180 );
q5_DH = deg2rad(-q5_Jaco + 180 );
q6_DH = deg2rad(-q6_Jaco + (180 + 80) );

% Iteration Num
T = 50;

% Q array:
%   q = [q1_1, q1_2, ..., q1_t, ..., q1_T;
%        q2_1, q2_2, ..., q2_t, ..., q2_T;
%        q3_1, q3_2, ..., q3_t, ..., q3_T;
%        q4_1, q4_2, ..., q4_t, ..., q3_T;
%        q5_1, q5_2, ..., q5_t, ..., q4_T;
%        q6_1, q6_2, ..., q6_t, ..., q6_T]
q = zeros(6, T+1);
% Set an initial angle into q
q(:,1) = [q1_DH; q2_DH; q3_DH; q4_DH; q5_DH; q6_DH];

% P array:
%  p   = [p_1, p_2, ..., p_t, ..., p_T]
%  p_t = [x0_t, x1_t, x2_t, x3_t, x4_t, x5_t, x6_t;
%         y0_t, y1_t, y2_t, y3_t, y4_t, y5_t, y6_t;
%         z0_t, z1_t, z2_t, z3_t, z4_t, z5_t, z6_t]
p = zeros(3, 7, T+1);

% Calculate forward kinematics for initial angles
t = 1;
p(:, :, t) = fk_jaco(q(:, t));

% Setting for inverse kimematics solution
% Target position
% p7g = [0.6; 0.5; 0.4];
p7g = [0; 0; 0.4];

% Update weight for angle update in inverse kinematics interation 
c = 0.1;

% Iteration for inverse kimematics
for t = 1:T
    % Find a numerical Jacobian for hand of Jaco arm
    % p(:, 7, t) is a current hand position
    J = JacoArmHandNumericalJacobin( q(:,t), p(:, :, t) );
    
    % Joinit angles update
    q(:,t+1) = q(:, t) + c * pinv(J)*(p7g - p(:, 7, t));
    % Position update
    p(:, :, t+1) = fk_jaco(q(:, t+1));
end

% Draw an arm pose
figure(1);
hold on;
% Draw a target position
plot3(p7g(1), p7g(2), p7g(3), 'ro');
% Draw an arm pose
DrawJacoArmPose3D(p(:, :,  1));
DrawJacoArmPose3D(p(:, :,  1));
% for t = 1:10:T
%     DrawJacoArmPose3D(p(:, :,  t));
% end
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
