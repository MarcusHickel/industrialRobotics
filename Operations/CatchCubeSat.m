
[qMatrix, x, cubeSatTransMatrix, toolCentrePointMatrix] = FunCatchCubeSat;
 %Map tcp transform through movement

%%
for i = 1:size(qMatrix,1)
     
      robotCR10.model.animate(qMatrix(i,:));

      if x(i) > max(x) || i > 25 % Halfway is peak velo of TCP
         cubeSat.model.base = toolCentrePointMatrix(i); % Caught by arm
      else
         cubeSat.model.base = cubeSatTransMatrix(i);  % Flying towards
      end

    cubeSat.model.animate(0);
    pause(0.01);

end