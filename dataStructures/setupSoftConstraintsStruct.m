function [ softConstraints ] = setupSoftConstraintsStruct( )
%setupSoftConstraintsStruct creates a struct containing ...
%
%Input:
%  (none)
%
%Outputs:
%  * softConstraints [struct]: tbd

% This file is part of the CAESAR MPC Suite developed at 
% ABB Corporate Research (CHCRC.C1).
% It is distributed under the terms of the Eclipse Public License v1.0,
% see the file LICENSE in the root directory.
%
% Authors:         Joachim Ferreau, Helfried Peyrl, 
%                  Dimitris Kouzoupis, Andrea Zanelli
% Last modified:   14/7/2015


    softConstraints = struct(   'index',        [],         ...     % flag indicating which outputs are softened
                                'alongHorizon', Boolean.no, ...     % if yes, different slack vector for each time step
                                'linWeight',    [],         ...     % vector with linear weights (can also be a cell if alongHorizon == Boolean.yes)
                                'quadWeight',   []          ...     % vector with quadratic weights (can also be a cell if alongHorizon == Boolean.yes)
                                );

end
