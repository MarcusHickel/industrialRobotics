tr1 = SE3(transl(0.5, 0.15, 1));
tr2 = SE3(transl(0, -0.5, 0.2) * troty(-pi/2));

[qMatrix(1:50,:), xdot]= resolvedMotionRateControl(robotCR10,tr1,tr2);
tcpMatrix = robotCR10.model.fkine(qMatrix);


for i = 1:50
    tcpMatrix(i) = robotCR10.model.fkine(qMatrix(i,:))*SE3(transl(0,0,0.2));
end

for i = 1:size(qMatrix,1)
     
    robotCR10.model.animate(qMatrix(i,:));


    cubeSat.model.base = tcpMatrix(i);
    cubeSat.model.animate(0);

    pause(0.01);

end