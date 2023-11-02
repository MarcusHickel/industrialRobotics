function [qMatrix, xdot] = resolvedMotionRateControl(robot,tr1,tr2,q0,steps,deltaT,lambda)
%RMRC Resolved Motion Rate Control, Calculates a matrix of q values between 
% two transforms that results in a linear path. Will only work with 6dof
% inlcudes DLS 
% [qMatrix, xdot] = resolvedMotionRateControl(robot,tr1,tr2,q0,steps,deltaT,lambda)
% robot = robotclass
% tr1 = SE3
% q = inital q value
% steps = number of steps
% 

% This assigns a default value if none was given
if exist('q0','var') == 0
    q0 = zeros(1,6); % For DLS, smaller = more accurate 
end
if exist('steps','var') == 0
    steps = 50; % Leaving these as default for nw
end
if exist('deltaT','var') == 0
    deltaT = 0.05; % Discrete time step
end
if exist('lambda','var') == 0
    lambda = 0.001; % For DLS, smaller = more accurate 
end


T1 = tr1; 

% Remapping SE3 to trajectory
tr1 = [tr1.t' tr1.torpy];
tr2 = [tr2.t' tr2.torpy];

x = zeros(6,steps);  % Allocating memory
s = lspb(0,1,steps); % Create interpolation scalar
    for i = 1:steps
        x(:,i) = tr1*(1-s(i)) + s(i)*tr2; % Create trajectory
    end


qMatrix = nan(steps,6); % Allocate Memory
xdot = nan(6,steps-1);

qMatrix(1,:) = robot.model.ikcon(T1, q0); % Solve for inital joint angles

% Solve velocitities via RMRC
    for i = 1:steps-1
        xdot(:,i) = (x(:,i+1) - x(:,i))/deltaT;         % Calculate velocity at discrete time step
        J = robot.model.jacob0(qMatrix(i,:));           % Get the Jacobian at the current state
        dls = pinv(J.'*J + lambda*eye(6))*J';           % Calculate DLS
        qdot = dls*xdot(:,i);                                
        qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';  % Update next joint state
    end

% robot.model.plot(qMatrix,'trail','r-');
end

