function [J] = JacoArmWristNumericalJacobin(q, p)
%JacoArmWristNumericalJacobin Calculate numerical Jacobian 
%   for given joint angles of Jaco arm wrist
%   Input
%       q = [q1; q2; q3; q4; q5; q6] Joint angles[rad]
%       p = [p0; p1; p2; p3; p4; p5; p6]
%         = [x0, x1, x2, x3, x4, x5, x6; 
%            y0, y1, y2, y3, y4, y5, y6;
%            z0, z1, z2, z3, z4, z5, z6]
%       Wrist is at p5 = [x5; y5; z5] index is 6

% Small angle displacement for caluculating numerical Jacobian
Dq = 1e-9;

% Calculate numerical Jacobian
% Find positions as q changes a small amount of Dq
p_Dq1 = fk_jaco( q + [Dq; 0; 0; 0; 0; 0] );
p_Dq2 = fk_jaco( q + [0; Dq; 0; 0; 0; 0] );
p_Dq3 = fk_jaco( q + [0; 0; Dq; 0; 0; 0] );
p_Dq4 = fk_jaco( q + [0; 0; 0; Dq; 0; 0] );
p_Dq5 = fk_jaco( q + [0; 0; 0; 0; Dq; 0] );
p_Dq6 = fk_jaco( q + [0; 0; 0; 0; 0; Dq] );

% Current wrist position: p(:,6)
% Calculate a numerical Jacobian for hand of Jaco arm 
J = [(p_Dq1(:, 6)-p(:, 6))/Dq, ...
     (p_Dq2(:, 6)-p(:, 6))/Dq, ...
     (p_Dq3(:, 6)-p(:, 6))/Dq, ...
     (p_Dq4(:, 6)-p(:, 6))/Dq, ...
     (p_Dq5(:, 6)-p(:, 6))/Dq, ...
     (p_Dq6(:, 6)-p(:, 6))/Dq];
end
