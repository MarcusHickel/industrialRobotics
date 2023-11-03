function [qMatrixCR10, toolCentrePointMatrix] = FunMoveToOperationSpace(robotCR10,qlast)
    tr1 = SE3(transl(0.5, 0.15, 1));
    tr2 = SE3(transl(-0.236, -0.452, 0.3) * troty(-pi/2));
    
    q0 = deg2rad([-90 -10 -140 0 0 -30]); % Suggested position
    [qMatrixCR10(1:50,:), xdot]= resolvedMotionRateControl(robotCR10,tr1,tr2,q0);
    toolCentrePointMatrix = robotCR10.model.fkine(qMatrixCR10);
    
    
    for i = 1:50
        toolCentrePointMatrix(i) = robotCR10.model.fkine(qMatrixCR10(i,:))*SE3(transl(0,0,0.2)*trotx(pi));
    end
end