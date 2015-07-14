function [ A,B,f ] = linearizeOde( fcnName,NX,NU, x0,u0,t0,tEnd, integratorName,integratorTol )
%linearizeOde does something.
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
        integratorTol = 1e-4;
        if ( nargin < 8 )
            integratorName = 'ode45';
        end
    end
    
    % tolerance for perturbing ODE
    TOL = integratorTol / 1e2;
    
    A = zeros( NX,NX );
    B = zeros( NX,NU );

    
    % compute A with finite differences
    xEndNominal = integrateOde( fcnName,NX,NU, x0,u0,t0,tEnd, integratorName,TOL );
    
    for ii=1:NX
        
        ei = zeros( NX,1 ); ei(ii) = 1;
        xEndDisturbed1 = integrateOde( fcnName,NX,NU, x0+TOL*ei,u0, t0,tEnd, integratorName,TOL );
        %xEndDisturbed2 = integrateOde( fcnName,NX,NU, x0-TOL*ei,u0, t0,tEnd,TOL );
        
        %A(:,ii) = (xEndDisturbed1 - xEndDisturbed2) ./ (2*TOL);
        A(:,ii) = (xEndDisturbed1 - xEndNominal) ./ TOL;
        
    end

    
    % compute B with finite differences
    for ii=1:NU
        
        ei = zeros( NU,1 ); ei(ii) = 1;
        xEndDisturbed1 = integrateOde( fcnName,NX,NU, x0,u0+TOL*ei, t0,tEnd, integratorName,TOL );
        %xEndDisturbed2 = integrateOde( fcnName,NX,NU, x0,u0-TOL*ei, t0,tEnd,TOL );
        
        %B(:,ii) = (xEndDisturbed1 - xEndDisturbed2) ./ (2*TOL);
        B(:,ii) = (xEndDisturbed1 - xEndNominal) ./ TOL;
        
    end
    
    f = xEndNominal - A*x0 - B*u0;
    
end
