import numpy
import ctypes

name = "FORCESPRONLPsolver"
requires_callback = True
lib = "lib/FORCESPRONLPsolver.dll"
lib_static = "lib/FORCESPRONLPsolver_static.lib"
c_header = "include/FORCESPRONLPsolver.h"
nstages = 3

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (  9,   1),    9),
 ("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (  9,   1),    9)]

# Output                | Type    | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("u0"                  , ""               , ctypes.c_double, numpy.float64,     (  1,),    1)]

# Info Struct Fields
info = \
[('it', ctypes.c_int),
 ('it2opt', ctypes.c_int),
 ('res_eq', ctypes.c_double),
 ('res_ineq', ctypes.c_double),
 ('rsnorm', ctypes.c_double),
 ('rcompnorm', ctypes.c_double),
 ('pobj', ctypes.c_double),
 ('dobj', ctypes.c_double),
 ('dgap', ctypes.c_double),
 ('rdgap', ctypes.c_double),
 ('mu', ctypes.c_double),
 ('mu_aff', ctypes.c_double),
 ('sigma', ctypes.c_double),
 ('lsit_aff', ctypes.c_int),
 ('lsit_cc', ctypes.c_int),
 ('step_aff', ctypes.c_double),
 ('step_cc', ctypes.c_double),
 ('solvetime', ctypes.c_double),
 ('fevalstime', ctypes.c_double),
 ('solver_id', ctypes.c_int * 8)
]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(3, 2, 0, 3, 1, 1, 0, 0), 
	(3, 2, 0, 3, 3, 3, 0, 0), 
	(3, 0, 0, 3, 3, 3, 0, 0)
]