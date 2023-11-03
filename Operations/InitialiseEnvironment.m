clear all
clf

deltaT = 0.05; % The Time Step for the simulation


robotCR10 = DobotCR10;
robotCR10.model.base = eye(4); % Why isnt it intiatlly? 
robotUR3 = UR3([transl(-.8,-.6,0)]);
robotUR3.model.animate([0 0 0 0 0 0]);
bolt = Bolts(2);

robotCR10.model.plot(zeros(1,6),'notiles','nobase')
hold on
robotCR10.model.delay = 0;
cubeSat.model.delay = 0;
axis([-2 2 -2 2 -2 2])
cubeSat = CubeSatellite;
eStop = EStop(1);
eStop.EStopModel{1}.base = transl(1.3, -1.3, -0.1) * trotz(pi);
eStop.EStopModel{1}.animate(0)
serviceSat_h = PlaceObject('ServiceSat.ply');


view([135 35])