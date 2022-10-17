function sr_info = srInfoInit(sr)
%SRINFOINIT Extract important information from SpaceRobot object to a
%struct
%
%   Output a struct with fields:
%
%

    sr_info = struct();
    sr_info.Name = sr.Name;
    sr_info.nk = sr.NumBodies;
    sr_info.n = sr.NumActiveJoints;
    sr_info.N = 6 + sr_info.n;
    sr_info.BodyNames = sr.BodyNames;
    sr_info.RFunc = '';
    
    % Find joint config idx for each body
    sr_info.jnt_idx = arrayfun(@(name) sr.Bodies{sr.findBodyIdxByName(name)}.Joint.Q_id, sr.BodyNames);

    sr_info.A = cell(1, 2);
    sr_info.M = cell(1, 2);
    sr_info.P = cell(1, 2);
     
    % Bodies Matrices
    sr_info.A{1} = sr.Base.A;   
    sr_info.M{1} = sr.Base.M;
    sr_info.P{1} = sr.Base.P;       

    sr_info.A{2} = zeros(6, 6, sr_info.nk);
    sr_info.M{2} = zeros(6, 6, sr_info.nk);
    sr_info.P{2} = zeros(6, 1, sr_info.nk);

    for i=1:sr_info.nk
        sr_info.A{2}(:, :, i) = sr.Bodies{i}.A;
        sr_info.M{2}(:, :, i) = sr.Bodies{i}.M;
        sr_info.P{2}(:, :, i) = sr.Bodies{i}.P;
    end  
end

