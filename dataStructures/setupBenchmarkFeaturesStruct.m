function [ benchmarkFeatures ] = setupBenchmarkFeaturesStruct( )
%setupBenchmarkFeaturesStruct creates a struct containing ...
%
%Input:
%  (none)
%
%Outputs:
%  * benchmarkFeatures [struct]: tbd

% This file is part of the CAESAR MPC Suite developed at 
% ABB Corporate Research (CHCRC.C1).
% It is distributed under the terms of the Eclipse Public License v1.0,
% see the file LICENSE in the root directory.
%
% Authors:         Joachim Ferreau, Helfried Peyrl
% Last modified:   30/11/2015


    benchmarkFeatures = struct( 'nQP',            	0,                  ...    % number of QP instances
                                'nX',               0,                  ...    % number of states
                                'nY',               0,                  ...    % number of outputs
                                'nU',               0,                  ...    % number of inputs
                                'nI',               0,                  ...    % length of prediction horizon
                                'nCH',              0,                  ...    % length of control horizon
                                'nInputBounds',   	0,                  ...    % number of input bounds
                                'nOutputBounds',   	0,                  ...    % number of state/output bounds
                                'nPolyConstr',   	0,                  ...    % number of polytopic constraints
                                'isStable',        	Boolean.yes,        ...    % number of polytopic constraints
                                'origin',        	Origin.undefined    ...    % origin of example: academic/random/industrial
                                );

end
