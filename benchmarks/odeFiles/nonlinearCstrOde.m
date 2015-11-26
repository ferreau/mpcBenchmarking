function [ dx ] = nonlinearCstrOde( ~,y )
%nonlinearCstrOde does something.
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


    NX = 4;
    NU = 2;
    
    x = y( 1:NX );
    u = y( NX+1:NX+NU );
    
    dx = zeros( NX+NU,1 );

    %% constants
    TIMEUNITS_PER_HOUR = 3600;
    
    k10 =  1.287e12;
    k20 =  1.287e12;
    k30 =  9.043e09;
    E1  =  -9758.3;
    E2  =  -9758.3;
    E3  =  -8560.0;
    H1  =      4.2;
    H2  =    -11.0;
    H3  =    -41.85;
    rho =      0.9342;
    Cp  =      3.01;
    kw  =   4032.0;
    AR  =      0.215;
    VR  =     10.0;
    mK  =      5.0;
    CPK =      2.0;

    cA0    =    5.1;
    theta0 =  104.9;
  
    %% differential states
    cA     = x(1);
    cB     = x(2);
    theta  = x(3);
    thetaK = x(4);
    
    
    %% intermediate states
 	k1 = k10*exp(E1/(273.15 +theta));
	k2 = k20*exp(E2/(273.15 +theta));
	k3 = k30*exp(E3/(273.15 +theta));

    
    %% differential equation
	dx(1) = (1/TIMEUNITS_PER_HOUR)*(u(1)*(cA0-cA) - k1*cA - k3*cA*cA); 
	dx(2) = (1/TIMEUNITS_PER_HOUR)*(-u(1)*cB + k1*cA - k2*cB); 
	dx(3) = (1/TIMEUNITS_PER_HOUR)*(u(1)*(theta0-theta) - (1/(rho*Cp)) *(k1*cA*H1 + k2*cB*H2 + k3*cA*cA*H3)+(kw*AR/(rho*Cp*VR))*(thetaK -theta)); 
	dx(4) = (1/TIMEUNITS_PER_HOUR)*((1/(mK*CPK))*(u(2) + kw*AR*(theta-thetaK)));

end
