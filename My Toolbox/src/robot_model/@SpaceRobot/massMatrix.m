function H = massMatrix(obj)
    %massMatrix Compute the mass matrix for given configuration
    %   H = massMatrix(ROBOT) returns the joint-space mass
    %   matrix, H, of ROBOT for ROBOT's home configuration.
    %
    %   H = massMatrix(ROBOT, Q) returns the joint-space mass
    %   matrix, H, of ROBOT for the given configuration Q.
    %
    %   Joint configuration Q must be specified as a pNum-by-1 or
    %   an 1-by-pNum vector, depending on the DataFormat property
    %   of ROBOT, where pNum is the position number of ROBOT.
    %
    %   The returned mass matrix H is a positive-definite symmetric
    %   matrix with size vNum-by-vNum, where vNum is the velocity
    %   number of ROBOT (degrees of freedom).
    %
    %   Examples:
    %       % Load example robot
    %       load ____
    %       H = sc.massMatrix();
    H = zeros(6+obj.NumActiveJoints);
    
    warning('Not yet implemented')
end