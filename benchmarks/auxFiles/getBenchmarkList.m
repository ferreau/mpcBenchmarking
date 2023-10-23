function [ benchmarkNames,benchmarkVariants,benchmarkFeatures ] = getBenchmarkList( )
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
% Last modified:   27/11/2015


    nBenchmarks = getNumBenchmarks( );

    benchmarkNames = cell( nBenchmarks,1 );
    benchmarkVariants = cell( nBenchmarks,1 );
    benchmarkFeatures = cell( nBenchmarks,1 );

    
    %curWarningState = adjustWarnLevel( WarnLevel.showNone );
    
    for ii=1:nBenchmarks

        curName = Benchmarks(ii);
        fprintf( '*' );
        
        try
            
            eval( ['benchmark = Benchmark_',char(curName),'();'] );
            benchmarkNames{ii}    = curName;
            benchmarkVariants{ii} = benchmark.variants;
            
            % determine features of each variant
            if ( nargout > 2 )
                
                curNumVariants = length(benchmarkVariants{ii});
                curBenchmarkFeatures = setupBenchmarkFeaturesStruct();
                curBenchmarkFeatures(2:curNumVariants) = setupBenchmarkFeaturesStruct();
                for jj=1:curNumVariants
                    curBenchmarkFeatures(jj) = determineBenchmarkFeatures( benchmarkNames{ii},benchmarkVariants{ii}(jj) );
                end
                benchmarkFeatures{ii} = curBenchmarkFeatures;
                
            end

        catch %err
        end

    end
    
    fprintf( '\n' );
    
    % restore warning state
    %warning( curWarningState );

end
