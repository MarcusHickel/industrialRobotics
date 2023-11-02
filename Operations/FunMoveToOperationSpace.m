function [qMatrixCR10, toolCentrePointMatrix] = FunMoveToOperationSpace(robotCR10,q0)
    tr1 = SE3(transl(0.5, 0.15, 1));
    tr2 = SE3(transl(0, -0.5, 0.2) * troty(-pi/2));
    
    [qMatrixCR10(1:50,:), xdot]= resolvedMotionRateControl(robotCR10,tr1,tr2,q0);
    toolCentrePointMatrix = robotCR10.model.fkine(qMatrixCR10);
    
    
    for i = 1:50
        toolCentrePointMatrix(i) = robotCR10.model.fkine(qMatrixCR10(i,:))*SE3(transl(0,0,0.2)*trotx(pi));
    end
end