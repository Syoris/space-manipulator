function scStruct = scToStruct(spaceRobot)
    scStruct = struct;

    scStruct.Name = spaceRobot.Name;
    scStruct.Base = spaceRobot.Base;
    scStruct.Links = spaceRobot.Links;
    scStruct.N = spaceRobot.NumActiveJoints;
    scStruct.H_symb = spaceRobot.H_symb;
    scStruct.Ttree_symb = spaceRobot.Ttree_symb;
    scStruct.CoMJacobsBase_symb = spaceRobot.CoMJacobsBase_symb;
    scStruct.H_symb = spaceRobot.H_symb;
    scStruct.C_symb = spaceRobot.C_symb;
    scStruct.Q_symb = spaceRobot.Q_symb;
end