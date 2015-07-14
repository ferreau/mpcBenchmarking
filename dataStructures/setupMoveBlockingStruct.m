function [ moveBlocking ] = setupMoveBlockingStruct( )
%setupMoveBlockingStruct creates a struct containing ...
%
%Input:
%  (none)
%
%Outputs:
%  * moveBlocking [struct]: tbd

% This file is part of the CAESAR MPC Suite developed at 
% ABB Corporate Research (CHCRC.C1).
% It is distributed under the terms of the Eclipse Public License v1.0,
% see the file LICENSE in the root directory.
%
% Authors:         Joachim Ferreau, Helfried Peyrl, 
%                  Dimitris Kouzoupis, Andrea Zanelli
% Last modified:   14/7/2015


    moveBlocking = struct(  'uIdx',     [],  ...    % indices of time steps where control action is allows to move (starting from 1)
                            'uCount',   [],  ...    % number of time steps with the same control (add up to ni)
                            'uIndVec',  [],  ...    % vector indicating the time steps where control action is allows to move
                            'uIndMat',  []   ...    % matrix indicating the time steps where control action is allows to move
                             );

end
