function [ deltaInput ] = setupDeltaInputStruct( )
%setupDeltaInputStruct creates a struct containing ...
%
%Input:
%  (none)
%
%Outputs:
%  * deltaInput [struct]: tbd

% This file is part of the CAESAR MPC Suite developed at 
% ABB Corporate Research (CHCRC.C1).
% It is distributed under the terms of the Eclipse Public License v1.0,
% see the file LICENSE in the root directory.
%
% Authors:         Joachim Ferreau, Helfried Peyrl, 
%                  Dimitris Kouzoupis, Andrea Zanelli
% Last modified:   14/7/2015


    deltaInput = struct(    'active',   Boolean.no, ...     % yes/no to enable/disable use of delta inputs
                            'weight',   [],         ...     % matrix, vector or scalar with DU weights
                            'dumax',    [],         ...     % upper bound on du (NIY)
                            'dumin',    []          ...     % lower bound on du (NIY)
                           );

end
