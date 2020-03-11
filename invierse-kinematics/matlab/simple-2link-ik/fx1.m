function x = fx1(q1)
%fx1 Forward kinematics function for x of the second joint
    L1 = 1.0;
    x = L1 * cos(q1);
end
