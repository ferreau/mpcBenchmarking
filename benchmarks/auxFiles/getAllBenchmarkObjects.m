function [ benchmarkObjects ] = getAllBenchmarkObjects( )
%getAllBenchmarkObjects does something.
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


    benchmarkNames = getBenchmarkList( );
    nBenchmarks = length( benchmarkNames );

    benchmarkObjects = cell( nBenchmarks,1 );


    %% count number of variants if required
    for ii=1:nBenchmarks

        curName = benchmarkNames{ii};
        eval( ['benchmarkObjects{ii} = Benchmark_',char(curName),'();'] );

    end

end
