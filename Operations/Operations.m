
InitialiseEnvironment;
%%

[qMatrixCR10{1}, cubeSatTransMatrix, toolCentrePointMatrix{1}, x] = FunCatchCubeSat(robotCR10);
[qMatrixCR10{2}, toolCentrePointMatrix{2}] = FunMoveToOperationSpace(robotCR10,qMatrixCR10{1}(end,:));
bolt.boltModel{1}.base = cubeSat.model.base.T * transl(0.025, 0, -0.045);% * troty(pi)
bolt.boltModel{2}.base = cubeSat.model.base.T * transl(-0.025, 0, -0.045);
bolt.boltModel{1}.animate(0)
bolt.boltModel{2}.animate(0)
qMatrixUR3 = FunOperateOnSatellite(cubeSat,bolt,robotUR3);

%%
for i = 1:size(qMatrixCR10{1},1)
     
      robotCR10.model.animate(qMatrixCR10{1}(i,:));

      if x(i) > max(x) || i > 25 % Halfway is peak velo of TCP
         cubeSat.model.base = toolCentrePointMatrix{1}(i); % Caught by arm
      else
         cubeSat.model.base = cubeSatTransMatrix(i);  % Flying towards
      end

    cubeSat.model.animate(0);
    pause(0.01);

end
for i = 1:size(qMatrixCR10{2},1)
     
    robotCR10.model.animate(qMatrixCR10{2}(i,:));


    cubeSat.model.base = toolCentrePointMatrix{2}(i);
    cubeSat.model.animate(0);

    pause(0.01);

end

j = 1;
%Animate
tic;
for i = 1:size(qMatrixUR3,1)
    if rem(i,50) == 0

        text = sprintf('Waypoint %d (%d) %0.2fs', j, i, toc);
        disp(text)
        j = j+1;
    end
    robotUR3.model.animate(qMatrixUR3(i,:));

    pause(.01);

end