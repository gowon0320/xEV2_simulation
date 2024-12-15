/*
 * AD tool to FORCESPRO Template - missing information to be filled in by createADTool.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2023. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif

#include "FORCESPRONLPsolver/include/FORCESPRONLPsolver.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif


#include "FORCESPRONLPsolver_casadi.h"



/* copies data from sparse matrix into a dense one */
static void FORCESPRONLPsolver_sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, FORCESPRONLPsolver_callback_float *data, FORCESPRONLPsolver_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((FORCESPRONLPsolver_float) data[j]);
        }
    }
}




/* AD tool to FORCESPRO interface */
extern solver_int32_default FORCESPRONLPsolver_adtool2forces(FORCESPRONLPsolver_float *x,        /* primal vars                                         */
                                 FORCESPRONLPsolver_float *y,        /* eq. constraint multiplers                           */
                                 FORCESPRONLPsolver_float *l,        /* ineq. constraint multipliers                        */
                                 FORCESPRONLPsolver_float *p,        /* parameters                                          */
                                 FORCESPRONLPsolver_float *f,        /* objective function (scalar)                         */
                                 FORCESPRONLPsolver_float *nabla_f,  /* gradient of objective function                      */
                                 FORCESPRONLPsolver_float *c,        /* dynamics                                            */
                                 FORCESPRONLPsolver_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FORCESPRONLPsolver_float *h,        /* inequality constraints                              */
                                 FORCESPRONLPsolver_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FORCESPRONLPsolver_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
                                 solver_int32_default iteration, /* iteration number of solver                         */
                                 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* AD tool input and output arrays */
    const FORCESPRONLPsolver_callback_float *in[4];
    FORCESPRONLPsolver_callback_float *out[7];
    

    /* Allocate working arrays for AD tool */
    
    FORCESPRONLPsolver_callback_float w[20];
	
    /* temporary storage for AD tool sparse output */
    FORCESPRONLPsolver_callback_float this_f = (FORCESPRONLPsolver_callback_float) 0.0;
    FORCESPRONLPsolver_callback_float nabla_f_sparse[3];
    
    
    FORCESPRONLPsolver_callback_float c_sparse[2];
    FORCESPRONLPsolver_callback_float nabla_c_sparse[6];
    
    
    /* pointers to row and column info for 
     * column compressed format used by AD tool */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for AD tool */
	in[0] = x;
	in[1] = p;
	in[2] = l;
	in[3] = y;

	if ((stage >= 0) && (stage < 2))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FORCESPRONLPsolver_objective_1(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = FORCESPRONLPsolver_objective_1_sparsity_out(1)[0];
			ncol = FORCESPRONLPsolver_objective_1_sparsity_out(1)[1];
			colind = FORCESPRONLPsolver_objective_1_sparsity_out(1) + 2;
			row = FORCESPRONLPsolver_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			FORCESPRONLPsolver_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		FORCESPRONLPsolver_dynamics_1(in, out, NULL, w, 0);
		if (c != NULL)
		{
			nrow = FORCESPRONLPsolver_dynamics_1_sparsity_out(0)[0];
			ncol = FORCESPRONLPsolver_dynamics_1_sparsity_out(0)[1];
			colind = FORCESPRONLPsolver_dynamics_1_sparsity_out(0) + 2;
			row = FORCESPRONLPsolver_dynamics_1_sparsity_out(0) + 2 + (ncol + 1);
			FORCESPRONLPsolver_sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if (nabla_c != NULL)
		{
			nrow = FORCESPRONLPsolver_dynamics_1_sparsity_out(1)[0];
			ncol = FORCESPRONLPsolver_dynamics_1_sparsity_out(1)[1];
			colind = FORCESPRONLPsolver_dynamics_1_sparsity_out(1) + 2;
			row = FORCESPRONLPsolver_dynamics_1_sparsity_out(1) + 2 + (ncol + 1);
			FORCESPRONLPsolver_sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

	}

	if ((stage >= 2) && (stage < 3))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FORCESPRONLPsolver_objective_3(in, out, NULL, w, 0);
		if (nabla_f != NULL)
		{
			nrow = FORCESPRONLPsolver_objective_3_sparsity_out(1)[0];
			ncol = FORCESPRONLPsolver_objective_3_sparsity_out(1)[1];
			colind = FORCESPRONLPsolver_objective_3_sparsity_out(1) + 2;
			row = FORCESPRONLPsolver_objective_3_sparsity_out(1) + 2 + (ncol + 1);
			FORCESPRONLPsolver_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

	}


    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((FORCESPRONLPsolver_float) this_f);
    }

    return 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
