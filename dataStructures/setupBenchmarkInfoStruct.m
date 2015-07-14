function [ benchmarkInfo ] = setupBenchmarkInfoStruct( )
%setupBenchmarkInfoStruct creates a struct containing ...
%
%Input:
%  (none)
%
%Outputs:
%  * benchmarkInfo [struct]: tbd

% This file is part of the CAESAR MPC Suite developed at 
% ABB Corporate Research (CHCRC.C1).
% It is distributed under the terms of the Eclipse Public License v1.0,
% see the file LICENSE in the root directory.
%
% Authors:         Joachim Ferreau, Helfried Peyrl, 
%                  Dimitris Kouzoupis, Andrea Zanelli
% Last modified:   14/7/2015


    benchmarkInfo = struct(   'ID',                 -1,                     ...    % problem's ID in benchmark collection
                              'name',               '',                     ...    % short problem name
                              'description',        '',                     ...    % concise description of the benchmark problem
                              'reference',          '',                     ...    % name of author or reference to literature
                              'origin',             Origin.undefined,       ...    % problem's origin
                              'conditioning',       Conditioning.undefined, ...    % conditioning of resulting QP problems
                              'feasibility',        Feasibility.undefined,  ...    % whether resulting QP problems are feasibile or not
                              'isOpenLoopStable',   Boolean.yes             ...    % flag indicating whether process model is open-loop stable
                              );

end
