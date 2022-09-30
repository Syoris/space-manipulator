%% NMPC Test with ACADOS
%% test of native matlab interface
clear all

load SR2.mat
sr.homeConfig;
sr.q = zeros(8, 1);

% model_path = fullfile(pwd,'..','pendulum_on_cart_model');
% addpath(model_path)

check_acados_requirements()

%% Discretization
N = 20;
T = 1; % time horizon length
x0 = sr.q;

nlp_solver = 'sqp'; % sqp, sqp_rti
qp_solver = 'partial_condensing_hpipm';
    % full_condensing_hpipm, partial_condensing_hpipm, full_condensing_qpoases, full_condensing_daqp
qp_solver_cond_N = 5; % for partial condensing
% integrator type
sim_method = 'erk'; % erk, irk, irk_gnsf

%% Model dynamics
model = pendulum_on_cart_model;
nx = model.nx;
nu = model.nu;
ny = size(model.cost_expr_y, 1);      % used in simulink example
ny_e = size(model.cost_expr_y_e, 1);


