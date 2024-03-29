function initMassMat(obj, d, simpH)
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

    d.Message = sprintf('Computing H Matrix...');

    H = sym(zeros(6 + obj.NumActiveJoints));

    Jacobians = obj.JacobsCoM_Base_symb;

    % Base
    Jv_B = Jacobians.(obj.BaseName)(1:3, :); % Base speed jacobian
    Jw_B = Jacobians.(obj.BaseName)(4:6, :); % Base rotation jacobian
    inertias = obj.getInertiaM('symbolic', true);

    bV = obj.Base.Mass * (Jv_B.' * Jv_B);
    bW = Jw_B.' * inertias.(obj.BaseName) * Jw_B;

    H = H + bV + bW;

    for i = 1:obj.NumBodies
        body = obj.Bodies{i};
        joint = body.Joint;

        Jv_i = Jacobians.(body.Name)(1:3, :);
        Jw_i = Jacobians.(body.Name)(4:6, :);
        I_i = inertias.(body.Name);

        iv_1 = body.Mass * (Jv_B.' * Jv_B);
        iv_2 = body.Mass * (Jv_B.' * Jv_i + Jv_i.' * Jv_B);
        iv_3 = body.Mass * (Jv_i.' * Jv_i);
        iw = Jw_i.' * I_i * Jw_i;

        H = H + iv_1 + iv_2 + iv_3 + iw;
    end

    obj.H_symb = H;

    if simpH

        for i = 1:size(H, 1)

            for j = 1:size(H, 1)
                d.Message = sprintf('Simplifying H... (%i, %i)', i, j);
                obj.H_symb(i, j) = simplify(obj.H_symb(i, j), 'IgnoreAnalyticConstraints', true, 'Seconds', 10);
            end

        end

    end

end
