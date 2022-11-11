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

        function [q_ddot, x_ddot, q_dot, x_dot, q_ee_ddot, x_ee_ddot, q_ee_dot, x_ee_dot] = stepImpl(obj, q, q_dot, q_ee, x_ee_dot, f0, n0, taum)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.                        
            u = [f0; n0; taum];
%             x = [q; q_dot];
            
            sr_info = obj.srInfo;

            N = sr_info.N;
            nk = sr_info.nk;
        
            %% --- Initialize states ---        
            % Inputs
%             q;
%             q_ee;
        
%             x_dot;
%             x_ee_dot;
            
            % Transforms
            x_dot = q_dot;
            x_dot(4:6) = omega2euler_local(q(4:6), q_dot(4:6)); % psi_b_dot = R_psi^-1*wb
        
            q_ee_dot = x_ee_dot;
            q_ee_dot(4:6) = euler2omega_inertial(q_ee(4:6), x_ee_dot(4:6)); % wee = R_psi*psi_ee_dot
        
            %% --- Compute accel ---
            % 1 - Kinematics
            Rb = zeros(3, 3);
            Ra = zeros(6, 6);
            Rm = zeros(3, 3 * nk);
            Tee = zeros(4, 4);
        
            [Rb, Ra, Rm, Tee] = feval(sr_info.RFunc, q);
            Rm = reshape(Rm, 3, 3, nk); % Split Rm to nk 3x3 arrays
        
            % 2 - Kinetics
            [t, t_dot, Omega, A, A_dot] = Kin(sr_info, q, q_dot, zeros(N, 1), {Rb, Ra, Rm});
        
            % 3 - C
            wb = t{1}(4:6);
            %     C = CorMat(sr_info, wb, Omega, A, A_dot, {Rb, Ra, Rm});
            %     h = C*q_dot;
            [tau, ~] = ID(sr_info, t, t_dot, Omega, A, A_dot);
            tau_b = blkdiag(Rb, zeros(3, 3)) * tau{1}; % Express base torque in intertial frame
            h = [tau_b; tau{2}];
        
            % 4 - Mass Mat
            D = MassM(sr_info, q, A);
            D_inv = D^ - 1;
        
            % 5 - FD
            q_ddot = D_inv * (u - h);
        
            % 6 - End Effector
            Ree = Tee(1:3, 1:3);
            Ree = blkdiag(Ree, Ree);
            J = Ree * Jacobian('endeffector', sr_info, A, {Rb, Ra});
            J_dot = Ree * Jacobian_dot('endeffector', sr_info, A, A_dot, {Rb, Ra}, wb, Omega);
        
            q_ee_ddot = J * q_ddot + J_dot * q_dot;
        
            %% --- Convert states ---
            x_ddot = q_ddot;
            x_ddot(4:6) = omega2euler_accel_local(q(4:6), x_dot(4:6), q_ddot(4:6)); % psi, psi_dot, wb_dot
        
            x_ee_ddot = q_ee_ddot;
            x_ee_ddot(4:6) = omega2euler_accel_inertial(q_ee(4:6), x_ee_dot(4:6), q_ee_ddot(4:6)); % omega2euler_accel(psi, psi_dot, wb_dot)
                  
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end

        function validateInputsImpl(~, q, q_dot, f0, n0, taum,u6,u7)
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
            num = 8;
        end
        
        function [out1, out2, out3, out4, out5, out6, out7, out8] = getOutputSizeImpl(obj)
            %getOutputSizeImpl Return size for each output port
            out1 = [obj.srInfo.N 1];
            out2 = [obj.srInfo.N 1];
            out3 = [obj.srInfo.N 1];
            out4 = [obj.srInfo.N 1];
            out5 = [6 1];
            out6 = [6 1];
            out7 = [6 1];
            out8 = [6 1];
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8] = getOutputDataTypeImpl(obj)
            %getOutputDataTypeImpl Return data type for each output port
            out1 = propagatedInputDataType(obj,1);
            out2 = propagatedInputDataType(obj,1);
            out3 = propagatedInputDataType(obj,1);
            out4 = propagatedInputDataType(obj,1);
            out5 = propagatedInputDataType(obj,1);
            out6 = propagatedInputDataType(obj,1);
            out7 = propagatedInputDataType(obj,1);
            out8 = propagatedInputDataType(obj,1);
        end

        function [out1, out2,out3,out4,out5,out6,out7,out8] = isOutputComplexImpl(~)
            %isOutputComplexImpl Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            out7 = false;
            out8 = false;
        end

        function [out1, out2,out3,out4,out5,out6,out7,out8] = isOutputFixedSizeImpl(~)
            %isOutputFixedSizeImpl Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
            out7 = true;
            out8 = true;
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
