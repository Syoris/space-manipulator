fprintf('### TWIST ###\n')
fprintf('--- Base ---\n')
t0_S = [t0_S(4:6); t0_S(1:3)];
if ~isequal(round(t0_S, 5), round(tb, 5))
        fprintf('ERROR\n')
        fprintf('[SPART, DeNOC (CoM in I)]\n')  
        disp([t0_S, tb])    
else
    fprintf('OK\n')
    fprintf('[SPART, DeNOC]\n')  
    disp([t0_S, tb]) 
end    

for i=1:3
    body = sc.Bodies{i};  
    fprintf('--- Body: %s ---\n', body.Name)
    
    t_S = [tm_S(4:6, i); tm_S(1:3, i)];

    [R, ~] = tr2rt(sc.getTransform(body.Name, 'symbolic', false));
    t_D = app_data.t_array(:, :, i);
    t_com = t_D(1:3) + skew(t_D(4:6)) * (body.CenterOfMass.');

    t_com_I = blkdiag(R, R) * [t_com; t_D(4:6)];
    
    if ~isequal(round(t_S, 5), round(t_com_I, 5))
        fprintf('ERROR\n')
        fprintf('[SPART, DeNOC (CoM in I)]\n')  
        disp([t_S, t_com_I])    
    else
        fprintf('OK\n')
        fprintf('[SPART, DeNOC (CoM in I)]\n')  
        disp([t_S, t_com_I]) 
    end 
end