R1 = [  0.0109, -0.0037,  0.9999; 
       -0.8851,  0.4653,  0.0114;
       -0.4653, -0.8851,  0.0018];
R2 = [ 0, 0, 1;
       0, 1, 0; 
       1, 0, 0];
q_est = acos(R1(2,1))+ pi/2;
R = [cos(q_est), -sin(q_est);
     sin(q_est),  cos(q_est)]
R * R1(2:3,1)
R * R1(2:3,2)
