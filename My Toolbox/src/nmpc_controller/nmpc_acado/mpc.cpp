/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

mxArray* ModelFcn_1_f = NULL;
mxArray* ModelFcn_1_jac = NULL;
mxArray* ModelFcn_1T  = NULL;
mxArray* ModelFcn_1X  = NULL;
mxArray* ModelFcn_1XA = NULL;
mxArray* ModelFcn_1U  = NULL;
mxArray* ModelFcn_1P  = NULL;
mxArray* ModelFcn_1W  = NULL;
mxArray* ModelFcn_1DX = NULL;
unsigned int ModelFcn_1NT  = 0;
unsigned int ModelFcn_1NX  = 0;
unsigned int ModelFcn_1NXA = 0;
unsigned int ModelFcn_1NU  = 0;
unsigned int ModelFcn_1NP  = 0;
unsigned int ModelFcn_1NW  = 0;
unsigned int ModelFcn_1NDX = 0;
unsigned int jacobianNumber_1 = -1;
double* f_store_1             = NULL;
double* J_store_1             = NULL;

void clearAllGlobals1( ){ 
    if ( f_store_1 != NULL ){
        f_store_1 = NULL;
    }

    if ( J_store_1 != NULL ){
        J_store_1 = NULL;
    }

    if ( ModelFcn_1_f != NULL ){
        mxDestroyArray( ModelFcn_1_f );
        ModelFcn_1_f = NULL;
    }

    if ( ModelFcn_1T != NULL ){
        mxDestroyArray( ModelFcn_1T );
        ModelFcn_1T = NULL;
    }

    if ( ModelFcn_1X != NULL ){
        mxDestroyArray( ModelFcn_1X );
        ModelFcn_1X = NULL;
    }

    if ( ModelFcn_1XA != NULL ){
        mxDestroyArray( ModelFcn_1XA );
        ModelFcn_1XA = NULL;
    }

    if ( ModelFcn_1U != NULL ){
        mxDestroyArray( ModelFcn_1U );
        ModelFcn_1U = NULL;
    }

    if ( ModelFcn_1P != NULL ){
        mxDestroyArray( ModelFcn_1P );
        ModelFcn_1P = NULL;
    }

    if ( ModelFcn_1W != NULL ){
        mxDestroyArray( ModelFcn_1W );
        ModelFcn_1W = NULL;
    }

    if ( ModelFcn_1DX != NULL ){
        mxDestroyArray( ModelFcn_1DX );
        ModelFcn_1DX = NULL;
    }

    if ( ModelFcn_1_jac != NULL ){
        mxDestroyArray( ModelFcn_1_jac );
        ModelFcn_1_jac = NULL;
    }

    ModelFcn_1NT  = 0;
    ModelFcn_1NX  = 0;
    ModelFcn_1NXA = 0;
    ModelFcn_1NU  = 0;
    ModelFcn_1NP  = 0;
    ModelFcn_1NW  = 0;
    ModelFcn_1NDX = 0;
    jacobianNumber_1 = -1;
}

void genericODE1( double* x, double* f, void *userData ){
    unsigned int i;
    double* tt = mxGetPr( ModelFcn_1T );
    tt[0] = x[0];
    double* xx = mxGetPr( ModelFcn_1X );
    for( i=0; i<ModelFcn_1NX; ++i )
        xx[i] = x[i+1];
    double* uu = mxGetPr( ModelFcn_1U );
    for( i=0; i<ModelFcn_1NU; ++i )
        uu[i] = x[i+1+ModelFcn_1NX];
    double* pp = mxGetPr( ModelFcn_1P );
    for( i=0; i<ModelFcn_1NP; ++i )
        pp[i] = x[i+1+ModelFcn_1NX+ModelFcn_1NU];
    double* ww = mxGetPr( ModelFcn_1W );
    for( i=0; i<ModelFcn_1NW; ++i )
        ww[i] = x[i+1+ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP];
    mxArray* FF = NULL;
    mxArray* argIn[]  = { ModelFcn_1_f,ModelFcn_1T,ModelFcn_1X,ModelFcn_1U,ModelFcn_1P,ModelFcn_1W };
    mxArray* argOut[] = { FF };

    mexCallMATLAB( 1,argOut, 6,argIn,"generic_ode" );
    double* ff = mxGetPr( *argOut );
    for( i=0; i<ModelFcn_1NX; ++i ){
        f[i] = ff[i];
    }
    mxDestroyArray( *argOut );
}

void genericJacobian1( int number, double* x, double* seed, double* f, double* df, void *userData  ){
    unsigned int i, j;
    double* ff;
    double* J;
    if (J_store_1 == NULL){
        J_store_1 = (double*) calloc ((ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP+ModelFcn_1NW)*(ModelFcn_1NX),sizeof(double));
        f_store_1 = (double*) calloc (ModelFcn_1NX,sizeof(double));
    }
    if ( (int) jacobianNumber_1 == number){
        J = J_store_1;
        ff = f_store_1;
        for( i=0; i<ModelFcn_1NX; ++i ) {
            df[i] = 0;
            f[i] = 0;
            for (j=0; j < ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP+ModelFcn_1NW; ++j){
                df[i] += J[(j*(ModelFcn_1NX))+i]*seed[j+1]; 
            }
        }
        for( i=0; i<ModelFcn_1NX; ++i ){
            f[i] = ff[i];
        }
    }else{
        jacobianNumber_1 = number; 
        double* tt = mxGetPr( ModelFcn_1T );
        tt[0] = x[0];
        double* xx = mxGetPr( ModelFcn_1X );
        for( i=0; i<ModelFcn_1NX; ++i )
            xx[i] = x[i+1];
        double* uu = mxGetPr( ModelFcn_1U );
        for( i=0; i<ModelFcn_1NU; ++i )
            uu[i] = x[i+1+ModelFcn_1NX];
        double* pp = mxGetPr( ModelFcn_1P );
        for( i=0; i<ModelFcn_1NP; ++i )
            pp[i] = x[i+1+ModelFcn_1NX+ModelFcn_1NU];
        double* ww = mxGetPr( ModelFcn_1W );
            for( i=0; i<ModelFcn_1NW; ++i )
        ww[i] = x[i+1+ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP];
        mxArray* FF = NULL;
        mxArray* argIn[]  = { ModelFcn_1_jac,ModelFcn_1T,ModelFcn_1X,ModelFcn_1U,ModelFcn_1P,ModelFcn_1W };
        mxArray* argOut[] = { FF };
        mexCallMATLAB( 1,argOut, 6,argIn,"generic_jacobian" );
        unsigned int rowLen = mxGetM(*argOut);
        unsigned int colLen = mxGetN(*argOut);
        if (rowLen != ModelFcn_1NX){
            mexErrMsgTxt( "ERROR: Jacobian matrix rows do not match (should be ModelFcn_1NX). " );
        }
        if (colLen != ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP+ModelFcn_1NW){
            mexErrMsgTxt( "ERROR: Jacobian matrix columns do not match (should be ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP+ModelFcn_1NW). " );
        }
        J = mxGetPr( *argOut );
        memcpy(J_store_1, J, (ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP+ModelFcn_1NW)*(ModelFcn_1NX) * sizeof ( double ));
        for( i=0; i<ModelFcn_1NX; ++i ) {
            df[i] = 0;
            f[i] = 0;
            for (j=0; j < ModelFcn_1NX+ModelFcn_1NU+ModelFcn_1NP+ModelFcn_1NW; ++j){
                df[i] += J[(j*(ModelFcn_1NX))+i]*seed[j+1];
            }
        }
        mxArray* FF2 = NULL;
        mxArray* argIn2[]  = { ModelFcn_1_f,ModelFcn_1T,ModelFcn_1X,ModelFcn_1U,ModelFcn_1P,ModelFcn_1W };
        mxArray* argOut2[] = { FF2 };
        mexCallMATLAB( 1,argOut2, 6,argIn2,"generic_ode" );
        ff = mxGetPr( *argOut2 );
        memcpy(f_store_1, ff, (ModelFcn_1NX) * sizeof ( double ));
        for( i=0; i<ModelFcn_1NX; ++i ){
            f[i] = ff[i];
        }
        mxDestroyArray( *argOut );
        mxDestroyArray( *argOut2 );
    }
}
#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState q1;
    DifferentialState q2;
    DifferentialState q3;
    DifferentialState q4;
    DifferentialState q5;
    DifferentialState q6;
    DifferentialState q7;
    DifferentialState q8;
    DifferentialState q_dot1;
    DifferentialState q_dot2;
    DifferentialState q_dot3;
    DifferentialState q_dot4;
    DifferentialState q_dot5;
    DifferentialState q_dot6;
    DifferentialState q_dot7;
    DifferentialState q_dot8;
    Control u1;
    Control u2;
    Control u3;
    Control u4;
    Control u5;
    Control u6;
    Control u7;
    Control u8;
    BMatrix acadodata_M1;
    acadodata_M1.read( "mpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "mpc_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << q1;
    acadodata_f2 << q2;
    acadodata_f2 << q3;
    acadodata_f2 << q4;
    acadodata_f2 << q5;
    acadodata_f2 << q6;
    acadodata_f2 << q7;
    acadodata_f2 << q8;
    acadodata_f2 << u1;
    acadodata_f2 << u2;
    acadodata_f2 << u3;
    acadodata_f2 << u4;
    acadodata_f2 << u5;
    acadodata_f2 << u6;
    acadodata_f2 << u7;
    acadodata_f2 << u8;
    Function acadodata_f3;
    acadodata_f3 << q1;
    acadodata_f3 << q2;
    acadodata_f3 << q3;
    acadodata_f3 << q4;
    acadodata_f3 << q5;
    acadodata_f3 << q6;
    acadodata_f3 << q7;
    acadodata_f3 << q8;
    ModelFcn_1T  = mxCreateDoubleMatrix( 1, 1,mxREAL );
    ModelFcn_1X  = mxCreateDoubleMatrix( 16, 1,mxREAL );
    ModelFcn_1XA = mxCreateDoubleMatrix( 0, 1,mxREAL );
    ModelFcn_1DX = mxCreateDoubleMatrix( 16, 1,mxREAL );
    ModelFcn_1U  = mxCreateDoubleMatrix( 8, 1,mxREAL );
    ModelFcn_1P  = mxCreateDoubleMatrix( 0, 1,mxREAL );
    ModelFcn_1W  = mxCreateDoubleMatrix( 0, 1,mxREAL );
    ModelFcn_1NT  = 1;
    ModelFcn_1NX  = 16;
    ModelFcn_1NXA = 0;
    ModelFcn_1NDX = 16;
    ModelFcn_1NP  = 0;
    ModelFcn_1NU  = 8;
    ModelFcn_1NW  = 0;
    DifferentialEquation acadodata_f1;
    ModelFcn_1_f = mxCreateString("srode");
    IntermediateState setc_is_1(25);
    setc_is_1(0) = autotime;
    setc_is_1(1) = q1;
    setc_is_1(2) = q2;
    setc_is_1(3) = q3;
    setc_is_1(4) = q4;
    setc_is_1(5) = q5;
    setc_is_1(6) = q6;
    setc_is_1(7) = q7;
    setc_is_1(8) = q8;
    setc_is_1(9) = q_dot1;
    setc_is_1(10) = q_dot2;
    setc_is_1(11) = q_dot3;
    setc_is_1(12) = q_dot4;
    setc_is_1(13) = q_dot5;
    setc_is_1(14) = q_dot6;
    setc_is_1(15) = q_dot7;
    setc_is_1(16) = q_dot8;
    setc_is_1(17) = u1;
    setc_is_1(18) = u2;
    setc_is_1(19) = u3;
    setc_is_1(20) = u4;
    setc_is_1(21) = u5;
    setc_is_1(22) = u6;
    setc_is_1(23) = u7;
    setc_is_1(24) = u8;
    ModelFcn_1_jac = NULL;
    CFunction cLinkModel_1( ModelFcn_1NX, genericODE1 ); 
    acadodata_f1 << cLinkModel_1(setc_is_1); 

    OCP ocp1(0, 0.2, 20);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.setModel( acadodata_f1 );


    ocp1.setNU( 8 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 0 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( HOTSTART_QP, NO );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HOTSTART_QP");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-10 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    options_flag = ExportModule1.set( CG_HARDCODE_CONSTRAINT_VALUES, NO );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: CG_HARDCODE_CONSTRAINT_VALUES");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 20 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "export_mpc" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");

    clearAllGlobals1( ); 

    clearAllStaticCounters( ); 
 
} 

