
InitialiseEnvironment;
IntJoyStickControl;
%% Calc

[qMatrixCR10{1}, cubeSatTransMatrix, toolCentrePointMatrix{1}, x] = FunCatchCubeSat(robotCR10);
[qMatrixCR10{2}, toolCentrePointMatrix{2}] = FunMoveToOperationSpace(robotCR10,qMatrixCR10{1}(end,:));



%% Animations : Catching
input('Push Enter to start')
for i = 1:size(qMatrixCR10{1},1)
      JoyEstop;
      robotCR10.model.animate(qMatrixCR10{1}(i,:));

      if x(i) > max(x) || i > 25 % Halfway is peak velo of TCP
         cubeSat.model.base = toolCentrePointMatrix{1}(i); % Caught by arm
      else
         cubeSat.model.base = cubeSatTransMatrix(i);  % Flying towards
      end
    robotCR10.model.fkine(qMatrixCR10{1}(i,:))
    cubeSat.model.animate(0);
    pause(0.01);

end
%% Animations : Moving to Operation zone
for i = 1:size(qMatrixCR10{2},1)
    JoyEstop;
    robotCR10.model.animate(qMatrixCR10{2}(i,:));


    cubeSat.model.base = toolCentrePointMatrix{2}(i);
    cubeSat.model.animate(0);
    robotCR10.model.fkine(qMatrixCR10{2}(i,:))
    pause(0.01);

end
%%  Operatate on satellite 
% Detect positions of bolts, Calcuations
bolt.boltModel{1}.base = cubeSat.model.base.T * transl(0.025, 0, -0.045);% * troty(pi)
bolt.boltModel{2}.base = cubeSat.model.base.T * transl(-0.025, 0, -0.045);
bolt.boltModel{1}.animate(0)
bolt.boltModel{2}.animate(0)
[qMatrixUR3, boltMatrix] = FunOperateOnSatellite(cubeSat,bolt,robotUR3);

j = 1;
%Animate
tic;
for i = 1:size(qMatrixUR3,1)
    JoyEstop;
    if rem(i,50) == 0

        text = sprintf('Waypoint %d (%d) %0.2fs', j, i, toc);
%         disp(text)
%         input('')
        j = j+1;
    end
    if i > 1*50 && 17*50 > i
        
        bolt.boltModel{1}.base = boltMatrix(:,:,i);
        bolt.boltModel{1}.animate(0)
        
    end
    robotUR3.model.animate(qMatrixUR3(i,:));
    robotUR3.model.fkine(qMatrixUR3(i,:))
    pause(.01);
    
end