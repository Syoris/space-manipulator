%% SC_6DoF_init
% 6 DoF SpaceRobot initialization from DH parameters. Using the 6 DoF arm
% designed by MDA. See [2016, Dubanchet] p.338

sc = SpaceRobot;
sc.Name = 'spaceRobot_6DoF';

% --- Manipulator Parameters ---
% ## Dynamic Parameters ##
% i   ρi      li      mi      Jx_i    Jy_i    Jz_i
% 1   2250    0.10    0.36    0.82    1.00    0.82
% 2   −       2.61    9.20    0.03    5.24    5.24
% 3   −       2.61    9.20    0.03    5.24    5.24 
% 4   −       0.10    0.36    1.00    0.82    0.82 
% 5   −       0.10    0.36    0.82    0.82    1.00 
% 6   −       0.50    1.78    0.04    0.04    0.01

length =    [0.1,   2.61,   2.62,   0.1,    0.1,    0.5,    0];
mass =      [0.36,  9.20,   9.20,   0.36,   0.36,   1.78,   0];
Jx =        [0.82,  0.03,   0.03,   1.00,   0.82,   0.04,   0];
Jy =        [1,     5.24,   5.24,   0.82,   0.82,   0.04,   0];
Jz =        [0.82,  5.24,   5.24,   0.82,   1,      0.01,   0];

% ## DH Parameters ##
% i     ai      αi      di      θi                  
% 1     l1      π/2     0.06    θ1                  
% 2     l2      0       0.10    θ2                  
% 3     l3      0       0.10    θ3                  
% 4     l4      −π/2    0.10    θ4                  
% 5     l5      π/2     0.14    θ5+π/2              
% 6     0       0       l6      θ6                  
% E     0       0       0       0                   

%          [a               alpha       d           theta]
dhparams = [length(1)       pi/2        0.06        0;      
            length(2)     	0	        0.10   	    0;      
            length(3)  	    0           0.10        0;
            length(4)  	    -pi/2       0.10        0;
            length(5)  	    pi/2        0.14        0;
            0  	            0           length(6)   0;
            0  	            0           0           0;];

% ## Link Parameters ##
jointsAxis = [[0 0 1];
              [0 0 1];
              [0 0 1];
              [0 0 1];
              [0 0 1];
              [0 0 1];
              [0 0 0]];

linksCoM = [[length(1)/2 0 0];
            [length(2)/2 0 0];
            [length(3)/2 0 0];
            [length(4)/2 0 0];
            [length(5)/2 0 0];
            [length(6)/2 0 0];
            [0 0 0];];

jointsHomePos = zeros(7, 1);

linkNames = {'Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Link6', 'endeffector'};
jointNames = {'jnt1', 'jnt2', 'jnt3', 'jnt4', 'jnt5', 'jnt6',  'jnt_ee'};

% --- Base Parameters ---
sizeBase = [2.6, 1.7, 1.8]; % Base [height, width, depth] [m], [z, x, y]
mBase = 786; % [kg]
comBase = [0 0 0];
intertiaBase = [401.5, 655.0, 632.1, 0, 0, 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]

sc.Base.Mass = mBase;
sc.Base.Inertia = intertiaBase;
sc.Base.HomeConf = [0; 0; 0; 0; 0; 0]; % [Rx; Ry; Rz; r; p; y]
sc.Base.ManipToBaseTransform = rt2tr(eye(3), [0; 0; sizeBase(1)/2]);




% --- Robot Initialization ---
nLinks = 7; % 6DoF + ee

linksVect = cell(1, nLinks);
jointsVect = cell(1, nLinks);

links = cell(nLinks,1);
joints = cell(nLinks,1);

for i=1:nLinks
    newLink = Link(linkNames{i});
     
    if any(jointsAxis(i, :))
        newJoint = Joint(jointNames{i}, 'revolute');
        newJoint.Axis = jointsAxis(i, :);
        newJoint.HomePosition = jointsHomePos(i);
    else
        newJoint = Joint(jointNames{i}, 'fixed');
    end

    newJoint.setFixedTransform(dhparams(i,:),'dh');
    newLink.Joint = newJoint;
    newLink.Mass = mass(i);
    newLink.Inertia = [Jx(i), Jy(i), Jz(i), 0, 0, 0];
    newLink.CenterOfMass = linksCoM(i, :);

    % Add link
    if i==1
        parent = sc.BaseName;
    else
        parent = linkNames(i-1);
    end
    
    links{i} = newLink;
    joints{i} = newJoint;
    sc.addLink(links{i}, parent);
end

%% --- Initialize Matrices ---
profile on
tic
sc.initKin();
sc.initDyn();
toc
profile viewer
profile off

sc.homeConfig;


%% Save Robot
fprintf('Saving robot\n')
clearvars -except sc
save 'Project/Models/SC_6DoF.mat'