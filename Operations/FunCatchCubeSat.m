function [qMatrixCR10, cubeSatTransMatrix, toolCentrePointMatrix, x] = FunCatchCubeSat(robotCR10)
% Find the speed of the Satellite
% Calcuate intersection point
% Generate trag that lines with Satellite travel vector and matches ...
% satellite speed for duration of closing the end effector

% Will to have to descope above, have everything on rails instead
    
    tr1 = SE3(transl(-0.5,0.15, 1));
    tr2 = SE3(transl(0.5, 0.15, 1));
    
    [qMatrixCR10(1:50,:), xdot]= resolvedMotionRateControl(robotCR10,tr1,tr2);
    % [qMatrix(51:100,:), xdotPlace]= resolvedMotionRateControl(robot,tr2,tr3,qMatrix(50,:));
    % [qMatrix(101:150,:), xdot]= resolvedMotionRateControl(robot,tr3,tr4,qMatrix(100,:));
    
    % Find the Max velocity of the TCP 
    x = zeros(size(qMatrixCR10,1),1);
    for i = 1:size(xdot,2)-1
        x(i) = robotCR10.model.fkine(0.05*xdot(:,i)).t(2);
    end
    
    maxTCPvelo = max(x); % Find max velocity of Tool centre point
    % tmatrix = nan(1,50);
    linearpath = linspace(-1,1,50);
    for i = 1:50
        cubeSatTransMatrix(i) = SE3(transl([linearpath(i),0.15,1.2]));% Create basic path for cubesat
        toolCentrePointMatrix(i) = robotCR10.model.fkine(qMatrixCR10(i,:))*SE3(transl(0,0,0.2));
    end
end