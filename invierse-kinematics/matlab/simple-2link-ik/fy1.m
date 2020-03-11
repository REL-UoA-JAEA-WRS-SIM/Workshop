function y = fy1(q1)
%fy1 Forward kinematics function for y of the second joint
    L1 = 1.0;
    y = L1 * sin(q1);
end

