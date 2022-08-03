clearvars
filename='SC_2DoF.urdf';

% sc = SpaceRobot('SC_2DoF.urdf'); % Not implemented
rTlbx = importrobot(filename);
show(rTlbx);
[rSpart, keys] = urdf2robot(filename);




