function [ ] = printBenchmarkFeatures( benchmarkNames,benchmarkVariants,benchmarkFeatures )
%printBenchmarkFeatures does something.
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
% Last modified:   30/11/2015


    nBenchmarks = length( benchmarkNames );
    if ( nBenchmarks <= 0 )
        disp( 'No benchmarks available.' );
        return;
    end

    
    %% print header
    nCols = 25 + 3 + 4 + 5 + 4+4+4 + 4+4 + 6 + 6 + 6*3 + 1;
    
    clc;
    disp( repmat( '-',1,nCols) );
    fprintf( '%23s  ','Name' );
    fprintf( '%3s | ','Var' );
    fprintf( '%4s | ','Orig' );
    fprintf( '%5s | ','nQP' );
    fprintf( '%4s','nX' );
    fprintf( '%4s','nY' );
    fprintf( '%4s | ','nU' );
    fprintf( '%4s','nPH' );
    fprintf( '%4s | ','nCH' );
    fprintf( '%6s | ','Constr' );
    fprintf( '%6s','Stable' );
    fprintf( '\n' );
    disp( repmat( '-',1,nCols) );

    
    %% print all benchmark with all variants
    for ii=1:nBenchmarks
        
        nVariants = length(benchmarkVariants{ii});
        for jj=1:nVariants
            
            curFeatures = benchmarkFeatures{ii}(jj);
            fprintf( '%25s',char(benchmarkNames{ii}) );
            fprintf( '%3d | ',benchmarkVariants{ii}(jj) ); tmp = char(curFeatures.origin);
            fprintf( '%4s | ',tmp(1:4) );
            fprintf( '%5d | ',curFeatures.nQP );
            fprintf( '%4d',curFeatures.nX );
            fprintf( '%4d',curFeatures.nY );
            fprintf( '%4d | ',curFeatures.nU );
            fprintf( '%4d',curFeatures.nI );
            fprintf( '%4d | ',curFeatures.nCH );
            fprintf( '%5s  | ',printBenchmarkFeatures_getConstrString(curFeatures) );
            fprintf( '%3s   ',printBenchmarkFeatures_getStableString(curFeatures) );
            fprintf( '\n' );
            
        end
        
        disp( repmat( '-',1,nCols) );
        
    end
    
end



function [ constrString ] = printBenchmarkFeatures_getConstrString( curFeatures )

    constrString = '';
    
    if ( curFeatures.nInputBounds > 0 )
        constrString = [constrString,'i/'];
    end
    if ( curFeatures.nOutputBounds > 0 )
        constrString = [constrString,'o/'];
    end
    if ( curFeatures.nPolyConstr > 0 )
        constrString = [constrString,'p'];
    end
    
    if ( strcmp( constrString(end),'/' ) )
        constrString(end) = [];
    end
end


function [ stableString ] = printBenchmarkFeatures_getStableString( curFeatures )

    if curFeatures.isStable
        stableString = 'yes';
    else
        stableString = 'no';
    end
    
end
