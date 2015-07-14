function [ nBenchmarks,nVariants ] = getNumBenchmarks( )
%getNumBenchmarks does something.
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


    nAllBenchmarks = length( enumeration('Benchmarks') );

    nBenchmarks = 0;
    nVariants = 0;

    %% count number of actually installed benchmark files
    for ii=1:nAllBenchmarks
        
        curName = Benchmarks(ii);
        try
            eval( ['benchmark = Benchmark_',char(curName),'();'] );
            nBenchmarks = nBenchmarks+1;
            nVariants = nVariants + benchmark.getNumVariants();
        catch %err
        end
        
    end
    
end
