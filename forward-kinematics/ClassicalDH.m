function [m] = ClassicalDH(alpha, a, d, q)
%DH returns homogeneous transformation matrix following DH method and parameters
m =  TransZ(d) * RotZ(q) * TransX(a) * RotX(alpha)
end