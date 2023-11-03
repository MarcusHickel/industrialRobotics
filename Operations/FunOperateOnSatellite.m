function [qMatrixUR3 boltMatrix] = FunOperateOnSatellite(cubeSat,bolt,robotUR3)
    %  CatchCubeSat;
    % Find transform of cubeSat
    % cubeSat.model.base;
    % bolt = Bolts(2);
    % calcuate screw positions basted on cubesat pos
    %  X = +-0.025m Y = 0m Z = 0.045m
    % Screws are 50mm apart and mirrored 
    bolt.boltModel{1}.base = cubeSat.model.base.T * transl(0.025, 0, -0.045);% Positions of bolt on cube sat
    bolt.boltModel{2}.base = cubeSat.model.base.T * transl(-0.025, 0, -0.045);
    bolt.boltModel{1}.animate(0)
    bolt.boltModel{2}.animate(0)
    % Find transforms of screw holding positions
    % bolt.boltModel{1}.base;
    % bolt.boltModel{2}.base;
    
    wypnt{1} = robotUR3.model.fkineUTS(robotUR3.model.getpos());
    % Calcuate Waypoint mission:
    % 1. Hover over ScrewA
    wypnt{2} = bolt.boltModel{1}.base.T * transl(0, 0, -0.010);
    % 2. Move on ScrewA
    wypnt{3} = bolt.boltModel{1}.base.T;
    % 3. Grab ScrewA (Gripper)
    %Gripper Close. Bolt is now attached to TCP
    
    % 4. Unscrew ScrewA ( Note Thread pitch is 1.5)
    % With a length of 7mm a mininum of 5 turns is needed
    % Maybe get the RMRC of moving out and then add the rotation by
    % directly editing the qMatrix?
    % Or
    % Create a transform every pi/2 turn
    for i = 4:9
        wypnt{i} = wypnt{i-1} * transl(0,0,-0.0015);
    end

    % 5. Hover over ScrewA Position 
    wypnt{10} = wypnt{9} * transl(0,0,-0.100);
    
    %Move directly over ScrewHolder. 200mm 50mm from base of UR3
    %Overwrite q values to just rotate the base?
    wypnt{11} = robotUR3.model.base.T * transl(0.02, 0.3, 0.2) * trotx(pi);
    
    % 6. Move to Holding position
    wypnt{12} = robotUR3.model.base.T * transl(0.02, 0.3, 0.010) * trotx(pi);
    % 7. Screw into holding position
    for i = 13:13+5
        wypnt{i} = wypnt{i-1} * transl(0,0,0.0015);
    end
    % 8. Float above holding position
    wypnt{19} = wypnt{17} * transl(0,0,-.3);
    % Repeat 1-7 for Screw B
    % Repeat 1-8 in reverse
    
    
    wypnt{20} = transl(-0.6, -0.794, 0.694) * trotx(-pi);
    
    % Visualise waypoints
    for i = 1:length(wypnt)
        
        wypnt{i} = SE3(wypnt{i});
        h_wpynttr = trplot(wypnt{i});
%         disp(i)
%         input('')
        try delete(h_wpynttr); catch; end
    end
    
    %Trag Calculation
    qMatrixUR3 = nan((length(wypnt)-1)*50,6);
    for i = 1:length(wypnt)-1 %steps
        try qMatrixUR3((i*50)-49:i*50,:) = resolvedMotionRateControl(robotUR3,wypnt{i},wypnt{i+1},qMatrixUR3((i*50)-50,:));
        catch; qMatrixUR3((i*50)-49:i*50,:) = resolvedMotionRateControl(robotUR3,wypnt{i},wypnt{i+1}); % disp('catch');
        end
     end
    
    % Control joint 6 to unscrew wyponts 3-8
    % This is pretty jank but the only way to make sure the RMRC function
    % doesnt choose another angle to spin itself
    for i = 4:9
    qMatrixUR3((i*50)-49:i*50,6) = linspace(qMatrixUR3((i*50)-49,6),qMatrixUR3((i*50)-49,6)-2*pi,50);
    end
    
    for i = 12:17
    qMatrixUR3((i*50)-49:i*50,6) = linspace(0,2*pi,50);
    end
       
    qf = [-2.0203   -1.5179    2.0781   -2.1310   -1.5708   -0.4496];
    i = 10;
    qMatrixUR3((i*50)-49:i*50,:) = jtraj(qMatrixUR3((i*50)-50,:),qf,50);
    
    qf = [0 -pi/2 0 -pi/2 0 0];
    i = 20;
    qMatrixUR3((i*50)-49:i*50,:) = jtraj(qMatrixUR3((i*50)-50,:),qf,50);
    
    boltMatrix = eye(4).*ones(4,4,20*50);
    for i = 1:18 % Transform of bolt
        boltMatrix(:,:,(i*50)-49:i*50) = robotUR3.model.fkineUTS(qMatrixUR3((i*50)-49:i*50,:));
    end
%     i = 9;
%     qMatrixUR3((i*50)-49:i*50,:) = qMatrixUR3((i*50)-49,:).*ones(50,1);
%     qMatrixUR3((i*50)-49:i*50,1) = linspace(qMatrixUR3((i*50)-49,1),qMatrixUR3((i*50)-49,1)+(pi/2),50);
    
%     wypnt{10} = robotUR3.model.fkine(qMatrixUR3((i*50)-49,:));
%     i = 11;
%     qMatrixUR3((i*50)-49:i*50,:) = resolvedMotionRateControl(robotUR3,wypnt{i},wypnt{i+1},qMatrixUR3((i*50)-50,:));
end