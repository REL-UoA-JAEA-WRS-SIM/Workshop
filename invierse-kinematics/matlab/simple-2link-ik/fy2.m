function  y = fy2(q1, q2)
%fy2 Forward kinematics function for y of hand
    L2 = 1.0;
    y = fy1(q1) + L2 * sin(q1 + q2);
end