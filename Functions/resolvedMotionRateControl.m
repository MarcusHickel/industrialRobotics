function [qMatrix] = resolvedMotionRateControl(robot,tr1,tr2,steps,deltaT)
%RMRC Resolved Motion Rate Control, creates a set of joint angles thats
% path is linear in movement
%   TODO: DLS

if exist('steps','var') == 0
    steps = 50; % Leaving these as default for nw
end
if exist('deltaT','var') == 0
    deltaT = 0.05; % Discrete time step
end
                                        
% T1 = [eye(3) [-0.4569 -0.1943 0.06655]'; zeros(1,3) 1];
T1 = robot.model.tool; %Initial transform
% T2 = [eye(3) [0 0 0.5]'; zeros(1,3) 1]; unused

 
% 3.7
x = zeros(6,steps); % Allocating memory
s = lspb(0,1,steps); % Create interpolation scalar
    for i = 1:steps
        x(:,i) = tr1*(1-s(i)) + s(i)*tr2; % Create trajectory
    end

% 3.8
qMatrix = nan(steps,6); % Allocate Memory

% 3.9
qMatrix(1,:) = robot.model.ikcon(T1, zeros(1,6));               % Solve for joint angles

% 3.10
    for i = 1:steps-1
        xdot = (x(:,i+1) - x(:,i))/deltaT;              % Calculate velocity at discrete time step
        J = robot.model.jacob0(qMatrix(i,:));           % Get the Jacobian at the current state
        qdot = pinv(J)*xdot;                            % Solve velocitities via RMRC
        qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';  % Update next joint state
    end

% robot.model.plot(qMatrix,'trail','r-');
end

