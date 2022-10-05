function [] = generate_code

Ts = 0.01;   % 1 ms
EXPORT = 1;

DifferentialState q(8) q_dot(8);
Control u(8);

f = acado.DifferentialEquation();
f.linkMatlabODE('srode_ocp_acado_mex');

%% SIMexport
% fprintf('----------------------------\n         SIMexport         \n----------------------------\n');
% 
% 
% sim = acado.SIMexport( Ts );
% % sim.setLinearInput( A1, B1 );
% 
% sim.setModel( f );
% 
% sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'   );
% sim.set( 'NUM_INTEGRATOR_STEPS',        1               );
% 
% if EXPORT
%     sim.exportCode('export_sim');
%     
%     cd export_sim
%     make_acado_integrator('../integrate_sr')
%     cd ..
% end
% 
% % x = [0; 0; pi/2; 0; 0];
% % xs = x; xs2 = x;
% % u = [10; -30]; 
% % N = 80;
% % t1 = 0; t2 = 0;
% % sim_input.u = u;
% % for i = 1:N
% %     sim_input.x = xs(:,end);
% %     tic
% %     states = integrate_robot(sim_input);
% %     t1 = t1+toc;
% %     xs(:,i+1) = states.value;
% % end

%% MPCexport
fprintf('----------------------------\n         MPCexport         \n----------------------------\n');
acadoSet('problemname', 'mpc');

N = 20;
n_XD = 16;
n_U = 8;
ocp = acado.OCP( 0.0, N*Ts, N );

W_mat = eye(n_XD+n_U,n_XD+n_U);
WN_mat = eye(n_XD,n_XD);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

h={q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8), u(1), u(2), u(3), u(4), u(5), u(6), u(7), u(8)};
hN = {q(1), q(2), q(3), q(4), q(5), q(6), q(7), q(8)};

ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );

% ocp.subjectTo( -100 <= controls <= 100 );
% ocp.subjectTo( 0.0 <= v <= 0.3 );
% ocp.subjectTo( -0.5 <= omega <= 0.5 );

% ocp.setLinearInput( A1,B1 );
ocp.setModel( f );

mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'NO'             	);
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				);
mpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES','NO' 				);
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING'   );
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        N                   );

if EXPORT
    mpc.exportCode( 'export_mpc' );
%     copyfile('../../../../../../external_packages/qpoases', 'export_mpc/qpoases')
    
    cd export_mpc
    make_sfunction('../sfunction_robot')
    cd ..
end

end
