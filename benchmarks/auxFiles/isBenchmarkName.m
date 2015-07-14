function [ answer,benchmark ] = isBenchmarkName( nameOrBenchmark )
%isBenchmarkName does something.
%
%Inputs:
%  (tbd)
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


    answer = 0;
    benchmark = [];

    if ( isa( nameOrBenchmark,'Benchmarks' ) > 0 )
        
        try
            eval( ['benchmark = Benchmark_',char(curName),'();'] );
            answer = 1;
            benchmark = nameOrBenchmark;
        catch %err
        end
        
    else

        try
            eval( ['benchmark = Benchmark_',nameOrBenchmark,'();'] );
            answer = 1;
        catch %err
        end
        
    end

end
