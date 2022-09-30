function sr_info = SR2_info() %#codegen
%SR2_INFO Summary of this function goes here
%   Detailed explanation goes here

%     load('SR2_data.mat', 'sr_info');

    sr_info = struct();
    sr_info.Name = 'SR2';
    sr_info.nk = 3;
    sr_info.n = 2;
    sr_info.N = 8;
    sr_info.BodyNames = {'Body1', 'Body2', 'endeffector'};
    sr_info.jnt_idx = [1; 2; -1];
    sr_info.A = cell(1, 2);
    sr_info.M = cell(1, 2);
    sr_info.P = cell(1, 2);
     
    % A
    sr_info.A{1} = [...    
                        1.0000         0         0         0         0         0;
                             0    1.0000         0         0         0    0.5000;
                             0         0    1.0000         0   -0.5000         0;
                             0         0         0    1.0000         0         0;
                             0         0         0         0    1.0000         0;
                             0         0         0         0         0    1.0000;
                    ];

    sr_info.A{2} = zeros(6, 6, sr_info.nk);
    sr_info.A{2}(:, :, 1) = [...        
                             1     0     0     0     0     0;
                             0     1     0     0     0     0;
                             0     0     1     0     0     0;
                             0     0     0     1     0     0;
                             0     0     0     0     1     0;
                             0     0     0     0     0     1;
                             ];
    
    sr_info.A{2}(:, :, 2) = [...   
                             1     0     0     0     0     0;
                             0     1     0     0     0     1;
                             0     0     1     0    -1     0;
                             0     0     0     1     0     0;
                             0     0     0     0     1     0;
                             0     0     0     0     0     1;
                             ];

    sr_info.A{2}(:, :, 3) = [...   
                            1.0000         0         0         0         0         0;
                                 0    1.0000         0         0         0    0.5000;
                                 0         0    1.0000         0   -0.5000         0;
                                 0         0         0    1.0000         0         0;
                                 0         0         0         0    1.0000         0;
                                 0         0         0         0         0    1.0000;
                            ];

    
    % M
    sr_info.M{1} = [...    
                   10.0000         0         0         0         0         0
                         0   10.0000         0         0         0         0
                         0         0   10.0000         0         0         0
                         0         0         0    9.3000         0         0
                         0         0         0         0    9.3000         0
                         0         0         0         0         0    9.3000
                    ];

    sr_info.M{2} = zeros(6, 6, sr_info.nk);
    sr_info.M{2}(:, :, 1) = [...        
                            5.0000         0         0         0         0         0
                                 0    5.0000         0         0         0    2.5000
                                 0         0    5.0000         0   -2.5000         0
                                 0         0         0    0.5000         0         0
                                 0         0   -2.5000         0    1.7500         0
                                 0    2.5000         0         0         0    1.7500
                             ];
    
    sr_info.M{2}(:, :, 2) = [...   
                            2.5000         0         0         0         0         0
                                 0    2.5000         0         0         0    0.6250
                                 0         0    2.5000         0   -0.6250         0
                                 0         0         0    0.1000         0         0
                                 0         0   -0.6250         0    0.2562         0
                                 0    0.6250         0         0         0    0.2562
                             ];

    sr_info.M{2}(:, :, 3) = [...   
                            1.0000         0         0         0         0         0
                                 0    1.0000         0         0         0         0
                                 0         0    1.0000         0         0         0
                                 0         0         0    0.0100         0         0
                                 0         0         0         0    0.0100         0
                                 0         0         0         0         0    0.0100
                            ];

    
    % P
    sr_info.P{1} = eye(6);

    sr_info.P{2} = zeros(6, 1, sr_info.nk);
    sr_info.P{2}(6, :, 1) = 1;
    sr_info.P{2}(6, :, 2) = 1;
    sr_info.P{2}(6, :, 3) = 1;
end

