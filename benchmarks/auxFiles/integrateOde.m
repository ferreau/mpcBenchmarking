function [ xEnd ] = integrateOde( fcnName,NX,NU, x0,u0,t0,tEnd, integratorName,integratorTol ) %#ok<INUSL>
%integrateOde does something.
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


    if ( nargin < 9 )
        integratorTol = 1e-6;
        if ( nargin < 8 )
            integratorName = 'ode45';
        end
    end
    
    odeOptions = odeset( 'RelTol',integratorTol,'AbsTol',integratorTol ); %#ok<NASGU>
    
    eval( ['[ T,Y ] = ',integratorName,'( @',fcnName,', [t0 tEnd], [x0;u0], odeOptions );'] );
    
    if ( abs( T(end)-tEnd ) > 1e-3 ) %#ok<COLND>
        error( [integratorName,' failed!'] );
    end
    
    xEnd = Y(end,1:NX)'; %#ok<COLND>

end
