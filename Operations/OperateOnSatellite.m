qMatrix = FunOperateOnSatellite;


j = 1;
%Animate
tic;
for i = 1:size(qMatrix,1)
    if rem(i,50) == 0

        text = sprintf('Waypoint %d (%d) %0.2fs', j, i, toc);
        disp(text)
        j = j+1;
    end
    robotUR3.model.animate(qMatrix(i,:));

    pause(.01);

end