function scStruct = scToStruct(SpaceRobot)
    scStruct = struct;

    scStruct.Name = SpaceRobot.Name;
    scStruct.Base = SpaceRobot.Base;
    scStruct.Links = SpaceRobot.Links;
    scStruct.H_symb = SpaceRobot.H_symb;
    scStruct.Ttree_symb = SpaceRobot.Ttree_symb;
    scStruct.CoMJacobsBase_symb = SpaceRobot.CoMJacobsBase_symb;
    scStruct.H_symb = SpaceRobot.H_symb;
    scStruct.C_symb = SpaceRobot.C_symb;
    scStruct.Q_symb = SpaceRobot.Q_symb;
end