classdef GetTransformBlock < matlab.System
    % GetTransformBlock GetTransform between robot body and inertial frame.
    % Output a transform  that converts points originally expressed in
    % sourceBodyName frame to be expressed in targetBodyName

    properties(Nontunable)
        spaceRobot;
        
        %spaceRobotStruct - SpaceRobot structure
        spaceRobotStruct = 0;
        
        %sourceBodyName - The name of the source body
        sourceBodyName = 0;
        
        %targetBodyName - The name of the target body
        targetBodyName = 0;
    end
    
    methods
        function obj = GetTransformBlock(varargin)
            %ForwardDynsBlock Constructor for Forward Dynamics block system object
            
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.spaceRobot = SpaceRobot(obj.spaceRobotStruct);
        end

        function tform = stepImpl(obj, q)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.     
            tform = obj.spaceRobot.getTransform(obj.sourceBodyName, 'TargetFrame',  'inertial', 'Config', q, 'symbolic', false);
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end

        function validateInputsImpl(~, q)
            %validateInputsImpl Validate inputs to the step method at initialization
            
            validateattributes(q,{'single','double'},{'vector'},'GetTransformBlock','Config');            
        end      

        function flag = isInputSizeMutableImpl(~,~)
            %isInputSizeMutableImpl Specify input size mutability
            %   Return false if input size cannot change
            %   between calls to the System object
            flag = false;
        end
        
        function flag = isInputDataTypeMutableImpl(~,~)
            %isInputDataTypeMutableImpl Specify input type mutability
            %   Return false if input data type cannot change
            %   between calls to the System object
            flag = false;
        end

        function num = getNumOutputsImpl(~)
            %getNumOutputsImpl Define total number of outputs
            num = 1;
        end
        
        function out = getOutputSizeImpl(~)
            %getOutputSizeImpl Return size for each output port
            out = [4 4];
        end

        function out = getOutputDataTypeImpl(obj)
            %getOutputDataTypeImpl Return data type for each output port
            out = propagatedInputDataType(obj,1);
        end

        function out = isOutputComplexImpl(~)
            %isOutputComplexImpl Return true for each output port with complex data
            out = false;
        end

        function out = isOutputFixedSizeImpl(~)
            %isOutputFixedSizeImpl Return true for each output port with fixed size
            out = true;
        end
    end

    methods(Access = protected, Static)
       function header = getHeaderImpl
           header = matlab.system.display.Header(mfilename('class'), ...
               'Title','GetTransform Block',...
               'Text','System object of SpaceRobot GetTransform');
        end

        function group = getPropertyGroupsImpl
            %getPropertyGroupsImpl Define property section(s) for System block dialog
            mainGroup = matlab.system.display.SectionGroup(...
                'Title','Parameters', ...
                'PropertyList',{'spaceRobotStruct', 'sourceBodyName', 'targetBodyName'});
            
            group = mainGroup;
        end

        function flag = showSimulateUsingImpl
            %showSimulateUsingImpl Return false if simulation mode hidden in System block dialog
            flag = true;
        end
    end

end
