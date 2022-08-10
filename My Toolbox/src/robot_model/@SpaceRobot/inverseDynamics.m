% TODO
function tau = inverseDynamics(obj, varargin)
    %inverseDynamics Compute required joint torques for desired motion.
    %   TAU = inverseDynamics(ROBOT) computes joint torques TAU
    %   required for ROBOT to statically hold its home
    %   configuration with no external forces applied.
    %
    %   TAU = inverseDynamics(ROBOT, Q) computes the required joint
    %   torques for ROBOT to statically hold the given
    %   configuration Q with no external forces applied.
    %
    %   TAU = inverseDynamics(ROBOT, Q, QDOT) computes the joint
    %   torques required for ROBOT given the joint configuration Q
    %   and joint velocities QDOT while assuming zero joint accelerations
    %   and no external forces. (Set Q = [] if the desired joint
    %   configuration is home configuration.)
    %
    %   TAU = inverseDynamics(ROBOT, Q, QDOT, QDDOT) computes the
    %   joint torques required for ROBOT given the joint
    %   configuration Q, joint velocities QDOT and joint accelerations
    %   QDDOT while assuming no external forces are applied. (Set
    %   QDOT = [] to indicate zero joint velocities.)
    %
    %   TAU = inverseDynamics(ROBOT, Q, QDOT, QDDOT, FEXT) computes
    %   the joint torques required for ROBOT given the joint
    %   configuration Q, joint velocities QDOT, joint accelerations
    %   QDDOT and the external forces FEXT. (Set QDDOT = [] to
    %   indicate zero joint accelerations.)
    %
    %   If ROBOT's DataFormat property is set to 'column', the
    %   input variables must be formatted as
    %   - Joint configuration, Q - pNum-by-1 vector
    %   - Joint velocities, QDOT - vNum-by-1 vector
    %   - Joint accelerations, QDDOT - vNum-by-1 vector
    %   - External forces, FEXT - 6-by-NB matrix
    %
    %   where:
    %   - pNum is the position number of ROBOT
    %   - vNum is the velocity number of ROBOT (degrees of freedom)
    %   - NB is the number of bodies in ROBOT
    %   - Each column of FEXT represents a wrench
    %     -- top 3 elements: moment
    %     -- bottom 3 elements: linear force
    %
    %   If the DataFormat property of ROBOT is set to 'row', then
    %   all the vectors/matrices above need to be transposed.
    %
    %   The returned joint torques TAU is either a vNum-by-1 or an
    %   1-by-vNum vector, depending on the DataFormat property.
    %
    %   Examples:
    %       % Load example robot
    %       load exampleRobots.mat
    %
    %       % Set lbr robot dynamics input data format to 'column'
    %       lbr.DataFormat = 'column';
    %
    %       % Generate a random configuration for lbr
    %       q = lbr.randomConfiguration
    %
    %       % Compute the required joint torques for lbr to
    %       % statically hold that configuration
    %       tau = inverseDynamics(lbr, q);
    %
    %   See also forwardDynamics, externalForce

%             narginchk(1,5);
%             [q, qdot, qddot, fext] = validateDynamicsFunctionInputs(obj.TreeInternal, true, varargin{:});
%             tau = robotics.manip.internal.RigidBodyTreeDynamics.inverseDynamics(obj.TreeInternal, q, qdot, qddot, fext);
%             tau = resultPostProcess(obj.TreeInternal, tau);
    warning('Not yet implemented')
end