function [ dx ] = chainOde( ~,y )
%chainOde does something.
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
    
    x = y( 1:NX ); x = x(:);
    u = y( NX+1:NX+NU ); u = u(:);
    
    dx = zeros( NX+NU,1 );

    %% constants
    m     = 0.03;
    d     = 1;
    L     = 0.0333;
    g     = [0,0,-9.81]';
    x0    = [0,0,0]';

    
    %% differential equation
    
    % compute spring forces
    F = zeros( 3*nMasses,1 );
    xPos = [ x0; x(1:3*nMasses+3) ];
    
    for ii=1:nMasses+1
        idx = (3*ii-2) : (3*ii);
        F(idx) = d * ( 1 - L / norm( xPos(idx+3) - xPos(idx),2 ) ) * ( xPos(idx+3) - xPos(idx) );
    end
    
    dx( 1:3*nMasses ) = x( (3*nMasses+3+1):(3*nMasses+3+3*nMasses) );
    dx( (3*nMasses+1):(3*nMasses+3) ) = u;
        
    for ii=1:nMasses
        idx = (3*ii-2) : (3*ii);
        dx(3*nMasses+3+idx) = ( F(idx+3) - F(idx) ) ./ m + g; % -2*x(3*nMasses+3+idx); % add friction to obtain steady-state
    end
	
end
