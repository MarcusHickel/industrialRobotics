function [qMatrix, xdot] = resolvedMotionRateControl(robot,tr1,tr2,steps,deltaT,lambda)
%RMRC Resolved Motion Rate Control, Calculates a matrix of q values between 
% two transforms that results in a linear path will only work with 6dof
% inlcudes DLS 

% This assigns a default value if none was given
if exist('steps','var') == 0
    steps = 50; % Leaving these as default for nw
end
if exist('deltaT','var') == 0
    deltaT = 0.05; % Discrete time step
end
if exist('lambda','var') == 0
    lambda = 0.001; % For DLS, smaller = more accurate 
end


T1 = transl(tr1(1),tr1(2),tr1(3)) ...
     * trotx(tr1(4)) ...
     * troty(tr1(5)) ...
     * trotz(tr1(6)); 


x = zeros(6,steps);  % Allocating memory
s = lspb(0,1,steps); % Create interpolation scalar
    for i = 1:steps
        x(:,i) = tr1*(1-s(i)) + s(i)*tr2; % Create trajectory
    end


qMatrix = nan(steps,6); % Allocate Memory


qMatrix(1,:) = robot.model.ikcon(T1, zeros(1,6)); % Solve for joint angles

% Solve velocitities via RMRC
    for i = 1:steps-1
        xdot = (x(:,i+1) - x(:,i))/deltaT;              % Calculate velocity at discrete time step
        J = robot.model.jacob0(qMatrix(i,:));           % Get the Jacobian at the current state
        dls = pinv(J.'*J + lambda*eye(6))*J';           % Calculate DLS
        qdot = dls*xdot;                                
        qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';  % Update next joint state
    end

% robot.model.plot(qMatrix,'trail','r-');
end

