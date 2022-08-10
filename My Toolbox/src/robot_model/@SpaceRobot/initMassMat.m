function initMassMat(obj)
    %massMatrix Compute the mass matrix for given configuration
    %   H = massMatrix(ROBOT) returns the joint-space mass
    %   matrix, H, of ROBOT for ROBOT's current configuration.
    
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
    %       H = obj.massMatrix();

    fprintf('\t Computing H Matrix\n');

    H = sym(zeros(6+obj.NumActiveJoints));

    Jacobians = obj.comJacobiansBase();
    
    % Base    
    Jv_B = Jacobians.(obj.BaseName)(1:3, :); % Base speed jacobian
    Jw_B = Jacobians.(obj.BaseName)(4:6, :); % Base rotation jacobian
    inertias = obj.getInertiaM();

    bV = obj.Base.Mass*(Jv_B'*Jv_B);
    bW = Jw_B'*inertias.(obj.BaseName)*Jw_B;

    H = H + bV + bW;

    for i=1:obj.NumLinks
        link = obj.Links{i};
        joint = link.Joint;
        
        % Compute for active joints
        if joint.Q_id ~= -1   
            Jv_i = Jacobians.(link.Name)(1:3, :);
            Jw_i = Jacobians.(link.Name)(4:6, :);
            I_i = inertias.(link.Name);
            
            iv_1 = link.Mass*(Jv_B'*Jv_B);
            iv_2 = 2*link.Mass*(Jv_B'*Jv_i);
            iv_2_2 = link.Mass*( Jv_B'*Jv_i + Jv_i'*Jv_B);
            iv_3 = link.Mass*(Jv_i'*Jv_i);
            iw = Jw_i' * I_i * Jw_i;
            
            H = H + iv_1 + iv_2_2 + iv_3 + iw;
        end
    end

    obj.Hsym = H;
end