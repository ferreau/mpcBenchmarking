function installBenchmarks( )
%installBenchmarks installs the MPC Benchmark Collection.
%
%Inputs:
%  (none)
%
%Outputs:
%  (none)

% This file is part of the CAESAR MPC Suite developed at 
% ABB Corporate Research (CHCRC.C1). 
% It is distributed under the terms of the Eclipse Public License v1.0,
% see the file LICENSE in the root directory.
%
% Authors:         Joachim Ferreau, Helfried Peyrl, 
%                  Dimitris Kouzoupis, Andrea Zanelli
% Last modified:   14/7/2015


    %% setup paths
    clc;
    addpath( genpath([pwd filesep 'benchmarks']) )
    addpath( genpath([pwd filesep 'dataStructures']) )
    
end
