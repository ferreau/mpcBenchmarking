function [ problem ] = Benchmark_dcMotor( variant )
%Benchmark_dcMotor does something.
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


% default variant
if ( nargin < 1 )
    variant = 1;
end


%% define MPC problem info
problem = Benchmark(variant);

% try to load benchmark from mat file
problem = problem.loadFromMatFile( 'dcMotor',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.dcMotor);
problem.info.name         = 'dcMotor';
problem.info.description  = 'Control of a position servomechanism: DC motor, a gear-box, an elastic shaft and a load.';
problem.info.reference    = 'A. Bemporad, E. Mosca: Fulfilling Hard Constraints in Uncertain Linear Systems by Reference Managing, Automatica, 34(4), pp. 451-461, 1998.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.yes;


%% define model parameters
Ls = 1;             % Shaft lenght
ds = 0.02;          % Shaft diameter
Js = 0;             % Shaft inertia (negligible)
Jm = 0.5;           % Motor inertia
Bm = 0.1;           % Motor viscous friction coefficient
R = 20;             % Resistance of the armature
kt = 10;            % Motor constant
r = 20;             % Gear ratio
k_theta = 1280.2;   % Torsional rigidity
Jl = 20*Jm;         % Load inertia
Bl = 25;            % Load viscous friction coefficient
Ts = 0.1;           % Samping time

Vmax =  220; 
Vmin = -220; 

Tmax =  78.5398;
Tmin = -78.5398;


%% define MPC problem data
Ac = [  0               1               0                   0                   ; 
       -k_theta/Jl     -Bl/Jl           k_theta/(r*Jl)      0                   ;
        0               0               0                   1                   ;
        k_theta/(r*Jm)  0               -k_theta/(r^2*Jm)  -(Bm + kt^2/R)/Jm   ];
          
Bc = [      0           ;
            0           ;
            0           ;
            kt/(R*Jm)   ];
            
Cc = eye(4)  ;
      

Dc = zeros(4,1);
 
problem.Ts = Ts;

sysd = c2d(ss(Ac,Bc,Cc,Dc),problem.Ts);
[problem.A, problem.B, problem.C,problem.D] = ssdata(sysd);
  
problem.umax =  Vmax;
problem.umin =  Vmin;


problem.M = [   k_theta    0  -k_theta/r   0   ; 0     1    0    1];
problem.N = [   0; 0];

theta_min = -100;
theta_max = 100; 
problem.dmax =  [Tmax theta_max]';
problem.dmin =  [Tmin theta_min]';

problem.P = [100 0 0 0; 0 0 0 0; 0 0 100 0; 0 0 0 0];
problem.Q = [100 0 0 0; 0 0 0 0; 0 0 100 0; 0 0 0 0];
problem.R = 0.0001;

problem.x0 = zeros(4,1);
problem.ni = 10;


problem.lookAhead = Boolean.no;


%% define control scenario

% reference signal properties
MaxValue     =  10; %  2
MinValue     = -10; % -2

problem.variants = [1 2];

switch variant
        
    case 1 % square wave reference signal
        
        problem.ni   = 10;
        problem.uIdx = [1 2];
        nPeriods     = 2;
        PeriodLength = 100;
        problem.yr   = generateSquareWave( PeriodLength,nPeriods,MaxValue,MinValue,r );
        
        
    case 2 % square wave reference signal
        
        problem.ni = 20;
        nPeriods     = 2;
        PeriodLength = 100;
        problem.yr   = generateSquareWave( PeriodLength,nPeriods,MaxValue,MinValue,r );
        
        
    case 3 % square wave reference signal
        
        problem.ni = 100;
        nPeriods     = 2;
        PeriodLength = 100;
        problem.yr   = generateSquareWave( PeriodLength,nPeriods,MaxValue,MinValue,r );
        
        
    case 4 % square wave reference signal, more rescrictive input constraints
        problem.umax =  30;
        problem.umin =  -30;
        problem.ni = 100;
        nPeriods     = 2;
        PeriodLength = 100;
        problem.yr   = generateSquareWave( PeriodLength,nPeriods,MaxValue,MinValue,r );
        
        
    case 5 % square wave reference signal , more polytopic input constraints (compare with variant 9 with a different R)
        
        MaxValue     =  3;
        MinValue     = -3;
        problem.dmax =  [5 theta_max]';
        problem.dmin =  [-5 theta_min]';
        problem.ni = 20;
        nPeriods     = 2;
        PeriodLength = 100;
        problem.yr   = generateSquareWave( PeriodLength,nPeriods,MaxValue,MinValue,r );
        
        
    case 9 % square wave reference signal
        problem.R = 1;
        MaxValue     =  3;
        MinValue     = -3;
        problem.dmax =  [5 theta_max]';
        problem.dmin =  [-5 theta_min]';
        problem.ni = 20;
        nPeriods     = 2;
        PeriodLength = 100;
        problem.yr   = generateSquareWave( PeriodLength,nPeriods,MaxValue,MinValue,r );
    
        
    otherwise
        error( 'Invalid variant number!' );

end

%problem.uIdx = 1:5; % Different control horizon

end

% Tstop=40*Ts;  

% REFERENCE: 
% http://www.sciencedirect.com/science/article/pii/S0005109897002136