clear all
clf

deltaT = 0.05; % The Time Step for the simulation


robotCR10 = DobotCR10;
robotUR3 = UR3([transl(-.6,-.6,0)]);
bolt = Bolts(2);

robotCR10.model.plot(zeros(1,6),'notiles','nobase')
hold on
robotCR10.model.delay = 0;
cubeSat.model.delay = 0;
axis([-2 2 -2 2 -2 2])
cubeSat = CubeSatellite;

serviceSat_h = PlaceObject('ServiceSat.ply');
view([135 35])