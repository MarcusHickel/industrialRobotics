clear all
clf

deltaT = 0.05; % The Time Step for the simulation


robotCR10 = DobotCR10;
robotUR3 = UR3([transl(-.6,-.6,0)]);


robotCR10.model.plot(zeros(1,6),'notiles','nobase')
hold on
robotCR10.model.delay = 0;
cubeSat.model.delay = 0;
axis([-2 2 -2 2 -2 2])
cubeSat = CubeSatellite;

serviceSat_h = PlaceObject('ServiceSat.ply');
view([135 32])

%%
% Find the speed of the Satellite
% Calcuate intersection point
% Generate trag that lines with Satellite travel vector and matches ...
% satellite speed for duration of closing the end effector

% Will to have to descope above, have everything on rails instead


%%

% tr1 = [-0.5 0.15 1 0 0 0];
% tr2 = [0.5 0.15 1 0 0 0];
%  tr3 = [0.2 0.15 0.4 0 0 0];
%  tr4 = [-0.2 0.15 0.4 0 0 0];

tr1 = SE3(transl(-0.5,0.15, 1));
tr2 = SE3(transl(0.5, 0.15, 1));

[qMatrix(1:50,:), xdot]= resolvedMotionRateControl(robotCR10,tr1,tr2);
% [qMatrix(51:100,:), xdotPlace]= resolvedMotionRateControl(robot,tr2,tr3,qMatrix(50,:));
% [qMatrix(101:150,:), xdot]= resolvedMotionRateControl(robot,tr3,tr4,qMatrix(100,:));

% Find the Max velocity of the TCP 
x = zeros(size(qMatrix,1),1);
for i = 1:size(xdot,2)-1
    x(i) = robotCR10.model.fkine(0.05*xdot(:,i)).t(2);
end

maxTCPvelo = max(x); % Find max velocity of Tool centre point
% tmatrix = nan(1,50);
linearpath = linspace(-1,1,50);
for i = 1:50
    tmatrix(i) = SE3(transl([linearpath(i),0.15,1.2]));% Create basic path for cubesat
    tcpMatrix(i) = robotCR10.model.fkine(qMatrix(i,:))*SE3(transl(0,0,0.2));
end
 %Map tcp transform through movement

%%
for i = 1:size(qMatrix,1)
     
      robotCR10.model.animate(qMatrix(i,:));

      if x(i) > max(x) || i > 25 % Halfway is peak velo of TCP
         cubeSat.model.base = tcpMatrix(i); % Caught by arm
      else
         cubeSat.model.base = tmatrix(i);  % Flying towards
      end

    cubeSat.model.animate(0);
    pause(0.01);

end

