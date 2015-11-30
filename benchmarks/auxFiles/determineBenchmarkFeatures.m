function [ benchmarkFeatures ] = determineBenchmarkFeatures( nameOrBenchmark,variant )
%determineBenchmarkFeatures does something.
%
%Inputs:
%  (none)
%
%Outputs:
%  (tbd)

% This file is part of the CAESAR MPC Suite developed at 
% ABB Corporate Research (CHCRC.C1).
% It is distributed under the terms of the Eclipse Public License v1.0,
% see the file LICENSE in the root directory.
%
% Authors:         Joachim Ferreau, Helfried Peyrl
% Last modified:   27/11/2015


    %% load benchmark
    try
        eval( ['benchmark = Benchmark_',char(nameOrBenchmark),'(',num2str(variant),');'] );
        benchmark = benchmark.initialize(); %#ok<NODEF>
        benchmark = benchmark.divide();
        benchmarkFeatures = setupBenchmarkFeaturesStruct();
    catch %err
        benchmarkFeatures = [];
        return;
    end
    
    
    %% determine features
    if ( length(benchmark) > 0 ) %#ok<ISMT>
        benchmarkFeatures.nQP = length(benchmark);
        benchmarkFeatures.nX = benchmark{1}.nx;
        benchmarkFeatures.nY = benchmark{1}.ny;
        benchmarkFeatures.nU = benchmark{1}.nu;
        benchmarkFeatures.nI = benchmark{1}.ni;
        
        if isempty(benchmark{1}.uIdx)
            benchmarkFeatures.nCH = benchmark{1}.ni;
        else
            benchmarkFeatures.nCH = length(benchmark{1}.uIdx);
        end
        
        benchmarkFeatures.nInputBounds = sum( isfinite(benchmark{1}.umin) | isfinite(benchmark{1}.umax) );
        benchmarkFeatures.nOutputBounds = sum( isfinite(benchmark{1}.ymin) | isfinite(benchmark{1}.ymax) );
        benchmarkFeatures.nPolyConstr = sum( isfinite(benchmark{1}.dmin) | isfinite(benchmark{1}.dmax) ) + ...
                                        sum( isfinite(benchmark{1}.dNmin) | isfinite(benchmark{1}.dNmax) );
                                    
        if ( sum(abs(eig(benchmark{1}.A)) > 1) > 0 )
            benchmarkFeatures.isStable = Boolean.no;
        else
            benchmarkFeatures.isStable = Boolean.yes;
        end
        
        benchmarkFeatures.origin = benchmark{1}.info.origin;
    end
    
end
