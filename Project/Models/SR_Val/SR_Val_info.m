function sr_info = SR_Val_info_struct()
%% General Infos
	sr_info = struct();
	sr_info.Name = 'SR_Val';
	sr_info.nk = 2;
	sr_info.n = 2;
	sr_info.N = 8;

	sr_info.A = cell(1, 2);
	sr_info.M = cell(1, 2);
	sr_info.P = cell(1, 2);
	sr_info.jnt_idx = [1 2 ];
	sr_info.RFunc = 'RFunc_SR_Val_mex';
	sr_info.BodyNames = {'Body1' 'endeffector' };

 %% A
	sr_info.A{1} = [...
		1.000000 0.000000 0.000000 -0.000000 0.500000 -0.000000 
		0.000000 1.000000 0.000000 -0.500000 -0.000000 0.000000 
		0.000000 0.000000 1.000000 0.000000 -0.000000 -0.000000 
		0.000000 0.000000 0.000000 1.000000 0.000000 0.000000 
		0.000000 0.000000 0.000000 0.000000 1.000000 0.000000 
		0.000000 0.000000 0.000000 0.000000 0.000000 1.000000 
	];

	sr_info.A{2} = zeros(6, 6, sr_info.nk);
	sr_info.A{2}(:, :, 1) = [...
		1.000000 0.000000 0.000000 -0.000000 0.000000 -0.000000 
		0.000000 1.000000 0.000000 -0.000000 -0.000000 0.000000 
		0.000000 0.000000 1.000000 0.000000 -0.000000 -0.000000 
		0.000000 0.000000 0.000000 1.000000 0.000000 0.000000 
		0.000000 0.000000 0.000000 0.000000 1.000000 0.000000 
		0.000000 0.000000 0.000000 0.000000 0.000000 1.000000 
	];
	sr_info.A{2}(:, :, 2) = [...
		1.000000 0.000000 0.000000 -0.000000 0.500000 -0.000000 
		0.000000 1.000000 0.000000 -0.500000 -0.000000 0.000000 
		0.000000 0.000000 1.000000 0.000000 -0.000000 -0.000000 
		0.000000 0.000000 0.000000 1.000000 0.000000 0.000000 
		0.000000 0.000000 0.000000 0.000000 1.000000 0.000000 
		0.000000 0.000000 0.000000 0.000000 0.000000 1.000000 
	];

 %% M
	sr_info.M{1} = [...
		500.000000 0.000000 0.000000 0.000000 0.000000 0.000000 
		0.000000 500.000000 0.000000 0.000000 0.000000 0.000000 
		0.000000 0.000000 500.000000 0.000000 0.000000 0.000000 
		0.000000 0.000000 0.000000 400.000000 0.000000 0.000000 
		0.000000 0.000000 0.000000 0.000000 400.000000 0.000000 
		0.000000 0.000000 0.000000 0.000000 0.000000 400.000000 
	];

	sr_info.M{2} = zeros(6, 6, sr_info.nk);
	sr_info.M{2}(:, :, 1) = [...
		0.360000 0.000000 0.000000 -0.000000 0.180000 -0.000000 
		0.000000 0.360000 0.000000 -0.180000 -0.000000 0.000000 
		0.000000 0.000000 0.360000 0.000000 -0.000000 -0.000000 
		-0.000000 -0.180000 0.000000 0.910000 0.000000 0.000000 
		0.180000 -0.000000 -0.000000 0.000000 1.090000 0.000000 
		-0.000000 0.000000 -0.000000 0.000000 0.000000 0.820000 
	];
	sr_info.M{2}(:, :, 2) = [...
		9.200000 0.000000 0.000000 -0.000000 0.000000 -0.000000 
		0.000000 9.200000 0.000000 -0.000000 -0.000000 11.500000 
		0.000000 0.000000 9.200000 0.000000 -11.500000 -0.000000 
		-0.000000 -0.000000 0.000000 0.030000 0.000000 0.000000 
		0.000000 -0.000000 -11.500000 0.000000 19.615000 0.000000 
		-0.000000 11.500000 -0.000000 0.000000 0.000000 19.615000 
	];

 %% P
	sr_info.P{1} = [...
		1.000000 0.000000 0.000000 0.000000 0.000000 0.000000 
		0.000000 1.000000 0.000000 0.000000 0.000000 0.000000 
		0.000000 0.000000 1.000000 0.000000 0.000000 0.000000 
		0.000000 0.000000 0.000000 1.000000 0.000000 0.000000 
		0.000000 0.000000 0.000000 0.000000 1.000000 0.000000 
		0.000000 0.000000 0.000000 0.000000 0.000000 1.000000 
	];

	sr_info.P{2} = zeros(6, 1, sr_info.nk);
	sr_info.P{2}(:, :, 1) = [...
		0.000000 
		0.000000 
		0.000000 
		0.000000 
		0.000000 
		1.000000 
	];
	sr_info.P{2}(:, :, 2) = [...
		0.000000 
		0.000000 
		0.000000 
		0.000000 
		0.000000 
		1.000000 
	];
end