classdef EStop < handle
    %BRICKS A class that creates a herd of robot EStops
	%   The EStops can be moved around randomly. It is then possible to query
    %   the current location (base) of the EStops.  

    % Copy and paste job from RobotCows. Yes this does mean you can plot
    % many step them to run around. I love my pet EStops
    % EStop Model from Artec Group inc.
    
    %removes the try catch warning
    %#ok<*TRYNC>    

    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 10;
    end
    
    properties
        %> Number of EStops
        EStopCount = 2;
        
        %> A cell structure of \c EStopCount EStop models
        EStopModel;
        
        %> paddockSize in meters
        paddockSize = [10,10];        
        
        %> Dimensions of the workspace in regard to the padoc size
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = EStop(EStopCount)
            if 0 < nargin
                self.EStopCount = EStopCount;
            end
            
            self.workspaceDimensions = [-self.paddockSize(1)/2, self.paddockSize(1)/2 ...
                                       ,-self.paddockSize(2)/2, self.paddockSize(2)/2 ...
                                       ,0,self.maxHeight];

            % Create the required number of EStops
            for i = 1:self.EStopCount
                self.EStopModel{i} = self.GetEStopModel(['EStop',num2str(i)]);
                % Random spawn
                basePose = SE3(SE2((2 * rand()-1) * self.paddockSize(1)/2 ...
                                         , (2 * rand()-1) * self.paddockSize(2)/2 ...
                                         , (2 * rand()-1) * 2 * pi));
                self.EStopModel{i}.base = basePose;
                
                 % Plot 3D model
                plot3d(self.EStopModel{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'scale',0.1,'noarrow','nowrist','notiles','noraise');
                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
            end

            axis equal
%             if isempty(findobj(get(gca,'Children'),'Type','Light'))
%                 camlight
%             end 
        end
        
        function delete(self)
            for index = 1:self.EStopCount
                handles = findobj('Tag', self.EStopModel{index}.name);
                h = get(handles,'UserData');
                try delete(h.robot); end
                try delete(h.wrist); end
                try delete(h.link); end
                try delete(h); end
                try delete(handles); end
            end
        end       
        
        %% PlotSingleRandomStep
        % Move each of the EStops forward and rotate some rotate value around
        % the z axis
        function PlotSingleRandomStep(self)
            for EStopIndex = 1:self.EStopCount
                % Move Forward
                self.EStopModel{EStopIndex}.base = self.EStopModel{EStopIndex}.base * SE3(SE2(0.2, 0, 0));
                animate(self.EStopModel{EStopIndex},0);
                
                % Turn randomly
                % Save base as a temp variable
                tempBase = self.EStopModel{EStopIndex}.base.T;
                rotBase = tempBase(1:3, 1:3);
                posBase = tempBase(1:3, 4);
                newRotBase = rotBase * rotz((rand-0.5) * 30 * pi/180);
                newBase = [newRotBase posBase ; zeros(1,3) 1];
                           
                % Update base pose
                self.EStopModel{EStopIndex}.base = newBase;
                animate(self.EStopModel{EStopIndex},0);                

                % If outside workspace rotate back around
                % Get base as temp
                tempBase = self.EStopModel{EStopIndex}.base.T;
                
                if tempBase(1,4) < self.workspaceDimensions(1) ...
                || self.workspaceDimensions(2) < tempBase(1,4) ...
                || tempBase(2,4) < self.workspaceDimensions(3) ...
                || self.workspaceDimensions(4) < tempBase(2,4)
                    self.EStopModel{EStopIndex}.base = self.EStopModel{EStopIndex}.base * SE3(SE2(-0.2, 0, 0)) * SE3(SE2(0, 0, pi));
                end
            end
            % Do the drawing once for each interation for speed
            drawnow();
        end    
        
        %% TestPlotManyStep
        % Go through and plot many random walk steps
        function TestPlotManyStep(self,numSteps,delay)
            if nargin < 3
                delay = 0;
                if nargin < 2
                    numSteps = 200;
                end
            end
            for i = 1:numSteps
                self.PlotSingleRandomStep();
                pause(delay);
            end
        end
    end
    
    methods (Static)
        %% GetEStopModel
        function model = GetEStopModel(name)
            if nargin < 1
                name = 'EStop';
            end
            [faceData,vertexData] = plyread('EStop.ply','tri');
            link1 = Link('alpha',0,'a',0,'d',0.005,'offset',0);
            model = SerialLink(link1,'name',name);
            
            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end
    end    
end