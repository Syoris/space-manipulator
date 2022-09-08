function scStruct = scToStruct(spaceRobot)
    scStruct = struct;

    scStruct.Name = spaceRobot.Name;
    scStruct.Base = spaceRobot.Base;
    scStruct.Links = spaceRobot.Links;

    scStruct.n = spaceRobot.NumActiveJoints;        % Number of active joints
    scStruct.N = spaceRobot.NumActiveJoints + 6;    % Number of active joints + base DoF
    
    scStruct.H_symb = spaceRobot.H_symb;
    scStruct.Ttree_symb = spaceRobot.Ttree_symb;
    
    scStruct.JacobsCoM_Base_symb = spaceRobot.JacobsCoM_Base_symb;
    scStruct.JacobsCoM_symb = spaceRobot.JacobsCoM_symb;

    scStruct.H_symb = spaceRobot.H_symb;
    scStruct.C_symb = spaceRobot.C_symb;
    scStruct.Q_symb = spaceRobot.Q_symb;

    scStruct.KinInitialized = spaceRobot.KinInitialized;
    scStruct.DynInitialized = spaceRobot.DynInitialized;
end