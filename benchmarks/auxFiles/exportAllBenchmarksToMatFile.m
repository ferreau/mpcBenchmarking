function [ ] = exportAllBenchmarksToMatFile( )
%exportAllBenchmarksToMatFile does something.
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


    benchmarkObjects = getAllBenchmarkObjects( );

    count = 0;
    
    for ii=1:length(benchmarkObjects)
        
        curBenchmark = benchmarkObjects{ii};
        curVariants  = curBenchmark.variants;
        
        for jj=1:length(curVariants)
            disp( ['Exporting ', curBenchmark.getLabel(curVariants(jj)),'.mat ...'] );
            curBenchmark.exportToMatFile( curVariants(jj) );
            count = count+1;
        end
                
    end
    
    disp( [num2str(count), ' benchmark variants successfully exported!'] );
    
end
