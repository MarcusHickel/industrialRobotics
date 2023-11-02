[qMatrix, tcpMatrix] = FunMoveToOperationSpace;

for i = 1:size(qMatrix,1)
     
    robotCR10.model.animate(qMatrix(i,:));


    cubeSat.model.base = tcpMatrix(i);
    cubeSat.model.animate(0);

    pause(0.01);

end