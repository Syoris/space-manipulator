classdef StatePropagationBlock < matlab.System
    % ForwardDynBlock FowardDyn of a SpaceRobot

    properties(Nontunable)
        spaceRobotStruct = 0;
        spaceRobot;
    end
    
    methods
        function obj = StatePropagationBlock(varargin)
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

        function [q_out, q_dot_out] = stepImpl(obj, q, q_dot)
            obj.spaceRobot.q = q;
            obj.spaceRobot.q_dot = q_dot;

            q_out = obj.spaceRobot.q;
            q_dot_out = obj.spaceRobot.q_dot;
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end

        function validateInputsImpl(~, q, q_dot)
            %validateInputsImpl Validate inputs to the step method at initialization
            
            validateattributes(q,{'single','double'},{'vector'},'StatePropagationBlock','q');
            validateattributes(q_dot,{'single','double'},{'vector'},'StatePropagationBlock','q_dot');            
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
            num = 2;
        end
        
        function [out1, out2] = getOutputSizeImpl(obj)
            %getOutputSizeImpl Return size for each output port
            out1 = [obj.spaceRobotStruct.N 1];
            out2 = out1;
        end

        function [out1, out2] = getOutputDataTypeImpl(obj)
            %getOutputDataTypeImpl Return data type for each output port
            out1 = propagatedInputDataType(obj,1);
            out2 = propagatedInputDataType(obj,1);
        end

        function [out1, out2]  = isOutputComplexImpl(~)
            %isOutputComplexImpl Return true for each output port with complex data
            out1 = false;
            out2 = false;
        end

        function [out1, out2] = isOutputFixedSizeImpl(~)
            %isOutputFixedSizeImpl Return true for each output port with fixed size
            out1 = true;
            out2 = true;
        end
    end

    methods(Access = protected, Static)
       function header = getHeaderImpl
           header = matlab.system.display.Header(mfilename('class'), ...
               'Title','State Propagation Block',...
               'Text','System object of to check joint limits');
        end

        function group = getPropertyGroupsImpl
            %getPropertyGroupsImpl Define property section(s) for System block dialog
            mainGroup = matlab.system.display.SectionGroup(...
                'Title','Parameters', ...
                'PropertyList',{'spaceRobotStruct'});
            
            group = mainGroup;
        end

        function flag = showSimulateUsingImpl
            %showSimulateUsingImpl Return false if simulation mode hidden in System block dialog
            flag = true;
        end
    end

end
