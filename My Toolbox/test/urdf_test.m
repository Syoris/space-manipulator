clearvars
filename='SC_2DoF.urdf';

sc = SpaceRobot('SC_2DoF.urdf');
rTlbx = importrobot(filename);
[rSpart, keys] = urdf2robot(filename);




