function TestDobot()
% from the basic DH parameters that we derive, create model of robot.  


% Link('theta',__,'d',__,'a',__,'alpha',__,'offset',__,'qlim',[ ... ])

            link(1) = Link('d',0.1765,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',-0.607,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.568,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d',0.125,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(5) = Link('d',0.125,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',0.1114,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
           
        
            link(2).offset = -pi/2;
            link(4).offset = -pi/2;



myRobot = SerialLink([link(1) link(2) link(3) link(4) link(5) link(6) ], 'name', 'DobotCR10')

q = zeros(1,6);

myRobot.plot(q)

myRobot.gravity
myRobot.base
myRobot.tool

%% Teach
myRobot.teach

end