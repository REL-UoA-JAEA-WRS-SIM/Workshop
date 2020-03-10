function  x = fx2(q1, q2)
%fx2 Forward kinematics function for x of hand
    L2 = 1.0;
    x = fx1(q1) + L2 * cos(q1 + q2);
end
