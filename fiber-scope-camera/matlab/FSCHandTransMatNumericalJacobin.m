function [J] = FSCHandTransMatNumericalJacobin(q, T06)
%FCSHandTransMatNumericalJacobin Calculate numerical Jacobian 
%   of homogeneous transformation matrix for given joint angles of FCS hand
%   Input
%       q = [q1; q2; q3; q4; q5; q6] Joint angles[rad]
%       T06: Homogeneous transformation matrix of hand

% Small angle displacement for caluculating numerical Jacobian
Dq = 1e-9;

% Calculate numerical Jacobian
% Find positions as q changes a small amount of Dq
[p, T01, T02, T03, T04, T05, T06_Dq1] = fk_fsc( q + [Dq; 0; 0; 0; 0; 0] );
[p, T01, T02, T03, T04, T05, T06_Dq2] = fk_fsc( q + [0; Dq; 0; 0; 0; 0] );
[p, T01, T02, T03, T04, T05, T06_Dq3] = fk_fsc( q + [0; 0; Dq; 0; 0; 0] );
[p, T01, T02, T03, T04, T05, T06_Dq4] = fk_fsc( q + [0; 0; 0; Dq; 0; 0] );
[p, T01, T02, T03, T04, T05, T06_Dq5] = fk_fsc( q + [0; 0; 0; 0; Dq; 0] );
[p, T01, T02, T03, T04, T05, T06_Dq6] = fk_fsc( q + [0; 0; 0; 0; 0; Dq] );

% Current hand position: p(:,7)
% Calculate a numerical Jacobian for hand of Jaco arm 
 J = [(T06_Dq1(1:3,1)-T06(1:3,1))/Dq, (T06_Dq2(1:3,1)-T06(1:3,1))/Dq, (T06_Dq3(1:3,1)-T06(1:3,1))/Dq, (T06_Dq4(1:3,1)-T06(1:3,1))/Dq, (T06_Dq5(1:3,1)-T06(1:3,1))/Dq, (T06_Dq6(1:3,1)-T06(1:3,1))/Dq;
      (T06_Dq1(1:3,2)-T06(1:3,2))/Dq, (T06_Dq2(1:3,2)-T06(1:3,2))/Dq, (T06_Dq3(1:3,2)-T06(1:3,2))/Dq, (T06_Dq4(1:3,2)-T06(1:3,2))/Dq, (T06_Dq5(1:3,2)-T06(1:3,2))/Dq, (T06_Dq6(1:3,2)-T06(1:3,2))/Dq;
      (T06_Dq1(1:3,3)-T06(1:3,3))/Dq, (T06_Dq2(1:3,3)-T06(1:3,3))/Dq, (T06_Dq3(1:3,3)-T06(1:3,3))/Dq, (T06_Dq4(1:3,3)-T06(1:3,3))/Dq, (T06_Dq5(1:3,3)-T06(1:3,3))/Dq, (T06_Dq6(1:3,3)-T06(1:3,3))/Dq;
      (T06_Dq1(1:3,4)-T06(1:3,4))/Dq, (T06_Dq2(1:3,4)-T06(1:3,4))/Dq, (T06_Dq3(1:3,4)-T06(1:3,4))/Dq, (T06_Dq4(1:3,4)-T06(1:3,4))/Dq, (T06_Dq5(1:3,4)-T06(1:3,4))/Dq, (T06_Dq6(1:3,4)-T06(1:3,4))/Dq;
     ];
end
