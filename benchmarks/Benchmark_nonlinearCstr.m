function [ problem ] = Benchmark_nonlinearCstr( variant )
%Benchmark_nonlinearCstr does something.
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
problem = problem.loadFromMatFile( 'nonlinearCstr',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.nonlinearCstr);
problem.info.name         = 'nonlinearCstr';
problem.info.description  = 'Start-Up of a Continuous Stirred Tank Reactor.';
problem.info.reference    = 'K.-U. Klatt, S. Engell: Ruehrkesselreaktor mit Parallel- und Folgereaktion, in: S. Engell (editor), Nichtlineare Regelung – Methoden, Werkzeuge, Anwendungen, VDI-Berichte Nr. 1026, pp. 101–108, VDI-Verlag, Duesseldorf, 1993.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.yes;


%% define MPC problem data

%{
problem.A = [   4.0988e-001 -1.7405e-002 -2.9159e-002 -6.4690e-003; ...
                1.6315e-001  5.5259e-001  7.7535e-003  1.9171e-003; ...
                1.4566e+000  6.9912e-001  6.6784e-001  2.2221e-001; ...
                8.9259e-001  4.2547e-001  6.2485e-001  3.1841e-001 ];
          
problem.B = [   4.5066e-002 -5.8176e-006; ...
               -1.1876e-002  1.8652e-006; ...
               -1.0863e-001  3.2969e-004; ...
               -6.6754e-002  1.1156e-003];
%}

% settings for linearizing ODE model
fcnName = 'nonlinearCstrOde';
NX = 4;
NU = 2;
x0 = [ 2.14; 1.09; 114.2; 112.9 ];
u0 = [ 14.19; -1113.5 ];
t0 = 0;
tEnd = 75;

[ problem.A,problem.B,problem.f ] = linearizeOde( fcnName,NX,NU, x0,u0,t0,tEnd );

problem.C  = eye( NX );
problem.D  = zeros( NX,NU );
    
problem.Ts = tEnd; % Sampling time [s]  


problem.umax = [ 35;     0];
problem.umin = [  3; -9000];
problem.ymax =  Inf * ones(NX,1);
problem.ymin = -Inf * ones(NX,1);

problem.Q = diag( [0.2,1.0,0.5,0.2] );
problem.R = diag( [0.5000,0.0000005] );

problem.ni = 20; 
%problem.x0 = [ 2.14; 1.09; 114.2; 112.9 ];
problem.x0 = [ 0; 0; 85; 85 ];


%% define control scenario
problem.variants = [1 2];

switch variant
    
    case 1
        
        for i = 1:problem(1).ni*5
            problem.yr{i} = [ 2.14; 1.09; 114.2; 112.9 ];
            problem.ur{i} = [ 14.19; -1113.5 ];
        end
        problem.simModel = [];
        
    case 2
        % simulate the non linear model
        for i = 1:problem(1).ni*5
            problem.yr{i} = [ 2.14; 1.09; 114.2; 112.9 ];
            problem.ur{i} = [ 14.19; -1113.5 ];
        end
        problem.simModel = fcnName;
        
    otherwise
        error( 'Invalid variant number!' );
end

end

% REFERENCE: 
% ...
