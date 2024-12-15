#ifdef __cplusplus
extern "C" {
#endif
    
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
										 solver_int32_default stage,     /* stage number (0 indexed)                            */
										 solver_int32_default iteration, /* iteration number of solver                          */
										 solver_int32_default threadID  /* Id of caller thread */);

#ifdef __cplusplus
} /* extern "C" */
#endif
