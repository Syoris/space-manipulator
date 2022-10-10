classdef ForwardDynBlock < matlab.System
    % ForwardDynBlock FowardDyn of a SpaceRobot

    properties(Nontunable)
        spaceRobotStruct = 0;
        spaceRobot;
    end
    
    methods
        function obj = ForwardDynsBlock(varargin)
            %ForwardDynsBlock Constructor for Forward Dynamics block system object
            
            % Support name-value pair arguments when constructing object
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.spaceRobot = SpaceRobot(obj.spaceRobotStruct);
        end

        function q_ddot = stepImpl(obj, q, q_dot, f0, n0, taum)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.            
            F = [f0; n0; taum];
            x = [q; q_dot];

            q_ddot = sr_state_func_mex(x, F);
            
%             q_ddot = sr_state_func(x, F);
%             q_ddot = sr_state_func_test(x, F);

            q_ddot = q_ddot(9:16);
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end

        function validateInputsImpl(~, q, q_dot, f0, n0, taum)
            %validateInputsImpl Validate inputs to the step method at initialization
            
            validateattributes(q,{'single','double'},{'vector'},'ForwardDynamicsBlock','Config');
            validateattributes(q_dot,{'single','double'},{'vector'},'ForwardDynamicsBlock','JointVel');
            validateattributes(f0,{'single','double'},{'vector'},'ForwardDynamicsBlock','BaseForce');
            validateattributes(n0,{'single','double'},{'vector'},'ForwardDynamicsBlock','BaseTorque');
            validateattributes(taum,{'single','double'},{'vector'},'ForwardDynamicsBlock','JointToruqe');
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
        
        function out = getOutputSizeImpl(obj)
            %getOutputSizeImpl Return size for each output port
            out = [obj.spaceRobotStruct.N 1];
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
               'Title','Forward Dynamics Block',...
               'Text','System object of SpaceRobot forward dynamics');
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
