function [ benchmarkNames,benchmarkVariants ] = getBenchmarkList( )
%getBenchmarkList does something.
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
% Authors:         Joachim Ferreau, Helfried Peyrl, 
%                  Dimitris Kouzoupis, Andrea Zanelli
% Last modified:   14/7/2015


    nBenchmarks = getNumBenchmarks( );

    benchmarkNames = cell( nBenchmarks,1 );
    benchmarkVariants = cell( nBenchmarks,1 );

    
    for ii=1:nBenchmarks

        curName = Benchmarks(ii);
        
        try
            eval( ['benchmark = Benchmark_',char(curName),'();'] );
            benchmarkNames{ii}    = curName;
            benchmarkVariants{ii} = benchmark.variants;
        catch %err
        end

    end

end
