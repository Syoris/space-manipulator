J1_s = sc2.computeJacobians('TargetFrame', 'base', 'symbolic', true);
J1_v = sc2.computeJacobians('TargetFrame', 'base', 'symbolic', false);

J2_s = sc2.comJacobiansBase('symbolic', true);
J2_v = sc2.comJacobiansBase('symbolic', false);

J1_s_v = J1_s;
J2_s_v = J2_s;

f = fields(J1_s);
for i=1:length(f)
    J1_s_v.(f{i}) = double(subs(J1_s_v.(f{i}), sc2.q_symb, sc2.q));   
    J2_s_v.(f{i}) = double(subs(J2_s_v.(f{i}), sc2.q_symb, sc2.q));   
end


%% Compare results
clc
f = fields(J1_s);
for i=1:length(f)
    linkName = f{i};

    fprintf('--- %s ---\n', linkName);

    fprintf('comJacobians Method\n');
%     fprintf('Symbolic to val:\n')
%     disp(J2_s_v.(linkName));
%     fprintf('Non symbolic:\n')
    disp(J2_v.(linkName));

    fprintf('computeJacobians Method\n');
%     fprintf('Symbolic to val:\n')
%     disp(J1_s_v.(linkName));
%     fprintf('Non symbolic:\n')
    disp(J1_v.(linkName));
end


