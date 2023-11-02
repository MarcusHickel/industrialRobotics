classdef Bolts < handle
    %Bolts A class that creates M10x1.5x7 bolts
    
    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 10;
    end
    
    properties
        %> Number of bolts
        boltCount = 2;
        
        %> A cell structure of \c boltCount bolt models
        boltModel;
        
        %> paddockSize in meters
        paddockSize = [1,1];        
        
        %> Dimensions of the workspace in regard to the padoc size
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = Bolts(boltCount)
            if 0 < nargin
                self.boltCount = boltCount;
            end
            
            self.workspaceDimensions = [-self.paddockSize(1)/2, self.paddockSize(1)/2 ...
                                       ,-self.paddockSize(2)/2, self.paddockSize(2)/2 ...
                                       ,0,self.maxHeight];

            % Create the required number of bolts
            for i = 1:self.boltCount
                self.boltModel{i} = self.GetBoltModel(['bolt',num2str(i)]);
                % Random spawn
                basePose = SE3(SE2((2 * rand()-1) * self.paddockSize(1)/2 ...
                                         , (2 * rand()-1) * self.paddockSize(2)/2 ...
                                         , (2 * rand()-1) * 2 * pi)); % Unnesscary 
                self.boltModel{i}.base = basePose;
                
                 % Plot 3D model
                plot3d(self.boltModel{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'notiles','noraise','noarrow');
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
            for index = 1:self.boltCount
                handles = findobj('Tag', self.boltModel{index}.name);
                h = get(handles,'UserData');
                try delete(h.robot); end
                try delete(h.wrist); end
                try delete(h.link); end
                try delete(h); end
                try delete(handles); end
            end
        end       
         
       
    end
    
    methods (Static)
        %% GetBoltModel
        function model = GetBoltModel(name)
            if nargin < 1
                name = 'Bolt';
            end
            [faceData,vertexData] = plyread('bolt.ply','tri');
            link1 = Link('alpha',pi,'a',0,'d',0.005,'offset',0);
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