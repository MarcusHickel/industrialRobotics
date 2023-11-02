classdef CubeSatellite < handle
    %CUBESATELLITE 
    %   Cube Satellite model
    
    properties
        model;
        workspaceDimensions;
    end
    
 methods
        %% ...structors
        function self = CubeSatellite()
            self.model = self.GetModel('cubesatA');
            basePose = eye(4);
            self.model.base = basePose; % Sets position to 0,0,0
            self.model.delay = 0; %So everything doesnt run slow    
            % Plot 3D model
            self.workspaceDimensions = [-0.1, 0.1 ...
                                       ,-0.1, 0.1 ...
                                       , -0.1  , 0.1];
            % Plot 3D model
            plot3d(self.model,0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist','notiles');;

        end
        
        function delete(self)
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try delete(h.robot); end
                try delete(h.wrist); end
                try delete(h.link); end
                try delete(h); end
                try delete(handles); end
        end
end
       
    
    methods (Static)
        %% GetModel
        function model = GetModel(name)
            if nargin < 1
                name = 'CubeSat';
            end
            [faceData,vertexData] = plyread('cubesatA.ply','tri');
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
