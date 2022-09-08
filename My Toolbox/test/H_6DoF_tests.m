% Simplify Results
N = sc.NumActiveJoints + 6;

Hsimp_symb = sym(zeros(N, N));

for j=1:N
    for k=1:N
        msg = sprintf('Simplifying H... (%i, %i)', j, k);
        sc.logger(msg, 'debug');
        Hsimp_symb(j, k) = simplify(sc.H_symb(j, k));
    end
end


%%
N = sc.NumActiveJoints + 6;
Hfuncs = cell(N, 1);

for i=1:N
    i
    Hfuncs{i} = matlabFunction(sc.H_symb(i, :), 'Vars', {sc.q_symb});
end

%%
Hfuncs2 = cell(2, 1);
Hfuncs2{1} = matlabFunction(sc.H_symb(1:6, :), 'Vars', {sc.q_symb});
Hfuncs2{2} = matlabFunction(sc.H_symb(7:end, :), 'Vars', {sc.q_symb});

%%
tic
H = zeros(N, N);
for i=1:N
    i
    H(i, :) = Hfuncs{i}(sc.q);
end
disp(H)
toc

%%
tic
H2 = zeros(N, N);
H2(1:6, :) = Hfuncs2{1}(sc.q);
H2(7:end, :) = Hfuncs2{2}(sc.q);
toc