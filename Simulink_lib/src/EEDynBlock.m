classdef EEDynBlock < matlab.System
    % ForwardDynBlock FowardDyn of a SpaceRobot

    properties(Nontunable)
        srInfo = 0;
        srInfoFunc = 0;
        
        srStateFunc = 0;
        srStateFuncMex = 0;
    end
    
    methods
        function obj = EEDynBlock(varargin)
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

        function [Xee, Xee_dot, Xee_ddot, Xee_dot_psi] = stepImpl(obj, q, q_dot, q_ddot, Xee_prev)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.            
            sr_info = obj.srInfo;
            
            % 1 - EE Position
            [Rb, Ra, Rm, Tee] = feval(sr_info.RFunc, q);          
            Rm = reshape(Rm, 3, 3, []); % Split Rm to nk 3x3 arrays

            [Ree, ree] = tr2rt(Tee);
            Ree_blk = blkdiag(Ree, Ree);

            psi_prev = Xee_prev(4:6);
            psi_ee = tr2rpy(Ree, 'zyx');

            if psi_ee(1)<0 && psi_prev(1)>0
                psi_ee(1) = psi_ee(1) + 2*pi;
            end

            Xee = [ree; psi_ee.'];

%             psi = zeros(3, 1);
%             temp = rotm2eul(R, 'ZYX');
%             psi(1) = temp(3);
%             psi(2) = temp(2);
%             psi(3) = temp(1);
            
            % 2 - Kinetics
            [t, ~, Omega, A, A_dot]  = Kin(sr_info, q, q_dot, zeros(sr_info.N, 1), {Rb, Ra, Rm});
            wb = t{1}(4:6);
            
            % 3 - J
            J = Ree_blk*Jacobian('endeffector', sr_info, A, {Rb, Ra});            
            J_dot = Ree_blk*Jacobian_dot('endeffector', sr_info, A, A_dot, {Rb, Ra}, wb, Omega);

            % 4 - Xee_dot
            Xee_dot = J*q_dot;  % Omega
            Xee_ddot = J*q_ddot + J_dot*q_dot;

            Xee_dot_psi = Xee_dot;
%             Xee_dot_psi(4:6) = omega2euler(Xee(4:6), Xee_dot(4:6));
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end

        function validateInputsImpl(~, q, q_dot, q_ddot, Xee_prev)
            %validateInputsImpl Validate inputs to the step method at initialization
            
            validateattributes(q,{'single','double'},{'vector'},'EEDynBlock','Joint config');
            validateattributes(q_dot,{'single','double'},{'vector'},'EEDynBlock','Joint velocities');
            validateattributes(q_ddot,{'single','double'},{'vector'},'EEDynBlock','Joint accels');
            validateattributes(Xee_prev,{'single','double'},{'vector'},'EEDynBlock','Xee_prev');
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
            num = 4;
        end
        
        function [out1, out2, out3, out4] = getOutputSizeImpl(~)
            %getOutputSizeImpl Return size for each output port
            out1 = [6 1];
            out2 = [6 1];
            out3 = [6 1];
            out4 = [6 1];
        end

        function [out1, out2, out3, out4] = getOutputDataTypeImpl(obj)
            %getOutputDataTypeImpl Return data type for each output port
            out1 = propagatedInputDataType(obj,1);
            out2 = propagatedInputDataType(obj,1);
            out3 = propagatedInputDataType(obj,1);
            out4 = propagatedInputDataType(obj,1);
        end

        function [out1, out2, out3, out4] = isOutputComplexImpl(~)
            %isOutputComplexImpl Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
        end

        function [out1, out2, out3, out4] = isOutputFixedSizeImpl(~)
            %isOutputFixedSizeImpl Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
        end
    end

    methods(Access = protected, Static)
       function header = getHeaderImpl
           header = matlab.system.display.Header(mfilename('class'), ...
               'Title','EE Dynamic Block',...
               'Text','System object of SpaceRobot end-effector dynamics computation');
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
