classdef ForwardDynBlock < matlab.System
    % ForwardDynBlock FowardDyn of a SpaceRobot

    properties(Nontunable)
        srInfo = 0;
        srInfoFunc = 0;
        
        srStateFunc = 0;
        srStateFuncMex = 0;
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
%             obj.spaceRobot = SpaceRobot(obj.spaceRobotStruct);
        end

        function [x_dot, q_ddot] = stepImpl(obj, q, q_dot, f0, n0, taum)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.            
            
            F = [f0; n0; taum];
            x = [q; q_dot];

            dx = obj.srStateFunc(x, F);
%             dx = obj.srStateFuncMex(x, F);
                        
            x_dot = dx(1:obj.srInfo.N);
            q_ddot = dx(obj.srInfo.N+1:end);
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
            num = 2;
        end
        
        function [out1, out2] = getOutputSizeImpl(obj)
            %getOutputSizeImpl Return size for each output port
            out1 = [obj.srInfo.N 1];
            out2 = [obj.srInfo.N 1];
        end

        function [out1, out2] = getOutputDataTypeImpl(obj)
            %getOutputDataTypeImpl Return data type for each output port
            out1 = propagatedInputDataType(obj,1);
            out2 = propagatedInputDataType(obj,1);
        end

        function [out1, out2] = isOutputComplexImpl(~)
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
               'Title','Forward Dynamics Block',...
               'Text','System object of SpaceRobot forward dynamics');
        end

        function group = getPropertyGroupsImpl
            %getPropertyGroupsImpl Define property section(s) for System block dialog
            mainGroup = matlab.system.display.SectionGroup(...
                'Title','Parameters', ...
                'PropertyList',{'srInfo', 'srInfoFunc', 'srStateFunc', 'srStateFuncMex'});
            
            group = mainGroup;
        end

        function flag = showSimulateUsingImpl
            %showSimulateUsingImpl Return false if simulation mode hidden in System block dialog
            flag = true;
        end
    end

end
