function [ xTraj ] = simChainOde( )
%simChainOde does something.
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
% Authors:         Joachim Ferreau, Helfried Peyrl
% Last modified:   2/12/2014


    nMasses = 9;

    NX = 3*nMasses + 3 + 3*nMasses; % pos + endPos + velocity
    NU = 3;
    
    x0 = zeros( NX,1 );
    u0 = zeros( NU,1 );
    u0 = [-1.5,1,1]';
    
    for ii=1:nMasses+1
        x0( 3*ii-2 ) = ii*0.5;
    end
    
    load chainSteadyState.mat;
    x0 = xSS;
    
    y0 = [x0;u0];

    %dx = chainOde( 0,y0 )
    for t=0:2
    
        [ T,yTraj ] = ode15s( @chainOde, t:0.1:t+1, y0 );
        xTraj = yTraj(:,1:NX);
        plotChain( T,xTraj );
        y0 = yTraj(end,:)';
        
    end
    
end
