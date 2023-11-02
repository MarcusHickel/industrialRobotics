classdef DobotCR10 < RobotBaseClass
    %% Dobot CR10  created using technical sheet

    properties(Access = public)              
        plyFileNameStem = 'DobotCR10';
    end
    
    methods
%% Define robot Function 
        function self = DobotCR10(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.model.delay = 0; 
%           self.PlotAndColourRobot(); Temp, uncomment when model is made   
        end

%% Create the robot model
        function CreateModel(self)   
            % Create the CR10 model by reading the technical manual
           
            link(1) = Link('d',0.1765,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',-0.607,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.568,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d',0.125,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(5) = Link('d',0.125,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',0.1114,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
           
        
            link(2).offset = -pi/2;
            link(4).offset = -pi/2;

            
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end