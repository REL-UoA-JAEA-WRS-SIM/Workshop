%
%   Inverse Kinematics for  Fiber Scope Camera
%       Author: Keitaro Naruse
%       Date: 2020-03-24
%

% Iteration Num
T = 200;

% Joint angle
%   q = [q1_1, q1_2, ..., q1_t, ..., q1_T+1;
%        q2_1, q2_2, ..., q2_t, ..., q2_T+1;
%        q3_1, q3_2, ..., q3_t, ..., q3_T+1;
%        q4_1, q4_2, ..., q4_t, ..., q3_T+1;
%        q5_1, q5_2, ..., q5_t, ..., q4_T+1;
%        q6_1, q6_2, ..., q6_t, ..., q6_T+1]
q = zeros(6, T+1);
% Joint position
%  p   = [p_1, p_2, ..., p_t, ..., p_T+1]
%  p_t = [x0_t, x1_t, x2_t, x3_t, x4_t, x5_t, x6_t;
%         y0_t, y1_t, y2_t, y3_t, y4_t, y5_t, y6_t;
%         z0_t, z1_t, z2_t, z3_t, z4_t, z5_t, z6_t]
p = zeros(3, 7, T+1);

% T6 array:
% T06 = [rXx, rYx, rZx, Dx;
%        rXy, rYy, rZy, Dy;
%        tXz, rYz, rZz, Dz;
%        0,   0,   0,   1]
% T6 = [T06_1, T06_2, ..., T06_t, ... T06_T+1]
T06 = zeros(4, 4, T+1);

% Initial time
t = 1;

% Set an initial angle into q
q(:,t) = [deg2rad(30); 
    deg2rad(30); 
    deg2rad(30); 
    deg2rad(30); 
    deg2rad(30); 
    deg2rad(30)
];
% Find initial positions
[p(:,:,t), T01, T02, T03, T04, T05, T06(:,:,t)] = fk_fsc(q(:,t));

% Target position of hand and wrist  
% Hand
p7g = [0.1; 0.1; 0.2];
% Wrist
p6g = [0.1; 0.0; 0.2];

% Update weight for angle update in inverse kinematics interation 
c = 0.1;
% Inverse kinematics interation
for t = 1:T
    % Find a numerical Jacobian for hand of Jaco arm
    J7 = FSCHandNumericalJacobin( q(:,t), p(:, :, t) );
    % Find a numerical Jacobian for wrist of Jaco arm
    J6 = FSCWristNumericalJacobin( q(:,t), p(:, :, t) );
    % Augumented Jacobian of hand and wrist
    J = [J7; J6];
    
    % Joinit angles update
    % Hand and wrist posiotin has index of 7 and 6, respectively
      q(:,t+1) = q(:, t) + c * pinv(J)*([p7g; p6g] - [p(:, 7, t); p(:, 6, t)]);
    % q(:,t+1) = q(:, t) + c * pinv(J7)*(p7g - p(:, 7, t));
    % Position update
    [p(:, :, t+1), T01, T02, T03, T04, T05, T06(:,:,t+1)] = fk_fsc(q(:, t+1));
end

% Display an arm pose with a hand frame
figure(1);
hold on;
% Draw a target position
plot3(p7g(1), p7g(2), p7g(3), 'ro');
plot3(p6g(1), p6g(2), p6g(3), 'go');
% Draw an initial arm pose
DrawFSCPose3DwithHandFrame(p(:,:,1), T06(:,:,1));
% Draw a final arm pose
DrawFSCPose3DwithHandFrame(p(:,:,T+1), T06(:,:,T+1));
hold off;
