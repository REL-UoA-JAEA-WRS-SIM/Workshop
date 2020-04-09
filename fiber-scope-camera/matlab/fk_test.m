% fk_test
figure(1);
hold on;
%for i = 0:30:360
    [p, T01, T02, T03, T04, T05, T06] = fk_fsc(...
        [deg2rad(0); 
         deg2rad(0); 
         deg2rad(0); 
         deg2rad(0); 
         deg2rad(0); 
         deg2rad(0)]);
    % Draw a final arm pose
    DrawFSCPose3DwithHandFrame(p, T06);
%end
hold off;
