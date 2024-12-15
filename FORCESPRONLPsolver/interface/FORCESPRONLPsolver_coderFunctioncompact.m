% FORCESPRONLPsolver : A fast customized optimization solver.
% 
% Copyright (C) 2013-2023 EMBOTECH AG [info@embotech.com]. All rights reserved.
% 
% 
% This software is intended for simulation and testing purposes only. 
% Use of this software for any commercial purpose is prohibited.
% 
% This program is distributed in the hope that it will be useful.
% EMBOTECH makes NO WARRANTIES with respect to the use of the software 
% without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
% PARTICULAR PURPOSE. 
% 
% EMBOTECH shall not have any liability for any damage arising from the use
% of the software.
% 
% This Agreement shall exclusively be governed by and interpreted in 
% accordance with the laws of Switzerland, excluding its principles
% of conflict of laws. The Courts of Zurich-City shall have exclusive 
% jurisdiction in case of any dispute.
% 
% [OUTPUTS] = FORCESPRONLPsolver(INPUTS) solves an optimization problem where:
% Inputs:
% - x0 - matrix of size [9x1]
% - xinit - matrix of size [2x1]
% - all_parameters - matrix of size [9x1]
% Outputs:
% - outputs - scalar
function [outputs] = FORCESPRONLPsolver(x0, xinit, all_parameters)
    
    [output, ~, ~] = FORCESPRONLPsolverBuildable.forcesCall(x0, xinit, all_parameters);
    outputs = coder.nullcopy(zeros(1,1));
    outputs(1:1) = output.u0;
end
