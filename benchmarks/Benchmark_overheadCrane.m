function [ problem ] = Benchmark_overheadCrane( variant )
%Benchmark_overheadCrane does something.
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
% Last modified:   26/11/2015


% default variant
if ( nargin < 1 )
    variant = 1;
end


%% define MPC problem info
problem = Benchmark(variant);

% try to load benchmark from mat file
problem = problem.loadFromMatFile( 'overheadCrane',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.overheadCrane);
problem.info.name         = 'overheadCrane';
problem.info.description  = 'Control of laboratory-scale overhead crane setup, featuring a fast moving cart and a winch mechanism. The aim is to perform optimized point-to-point motions with varying line length while respecting actuator limits.';
problem.info.reference    = 'M. Vukov, W. Van Loock, B. Houska, H.J. Ferreau, J. Swevers, M. Diehl: Experimental Validation of Nonlinear MPC on an Overhead Crane using Automatic Code Generation. ACC, Montreal, Canada, pp. 6264-6269, 2012.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.allFeasible;


%% define MPC problem data
problem.Ts = 0.01; % [s]  

% settings for linearizing ODE model
fcnName = 'overheadCraneOde';
NX = 8;
NU = 2;
t0 = 0;
tEnd = problem.Ts;
xSS = [ 0; 0; 0.9; 0; 0; 0; 0; 0 ];
uSS = [ 0; 0 ];

[ problem.A,problem.B,problem.f ] = linearizeOde( fcnName,NX,NU, xSS,uSS,t0,tEnd, 'ode23s',1e-6 );

problem.C  = eye( NX );
problem.D  = zeros( NX,NU );

problem.umax = [  100;  100 ];
problem.umin = [ -100; -100 ];
problem.ymax = [  Inf;  Inf;  Inf;  Inf;  Inf;  Inf;  10;  10 ];
problem.ymin = [ -Inf; -Inf; -Inf; -Inf; -Inf; -Inf; -10; -10 ];

problem.Q = diag( [ 56; 6; 115; 0.01; 10; 10; 1e-8; 1e-8 ] );
problem.R = diag( [ 1e-5; 1e-5 ] );

problem.ni   = 100;
problem.uIdx = [1:10:100]; % 10 equidistant control intervals, though reference mentions 12
problem.lookAhead = Boolean.no;

problem.x0 = xSS;


%% define control scenario
problem.variants = [1 2];

switch variant
    
    case 1
        % nonlinear simulation
        problem.simModel = fcnName;
        counter = 0;
        for i = 0 : problem.Ts : (2-problem.Ts)
            counter = counter+1;
            problem.yr{counter} = xSS;
            problem.ur{counter} = uSS;
        end
        for i = 2 : problem.Ts : (10-problem.Ts)
            counter = counter+1;
            problem.yr{counter} = [ 0.2; 0; 0.7; 0; 0; 0; 0; 0 ];
            problem.ur{counter} = [ 0; 0 ];
        end
        for i = 10 : problem.Ts : (18-problem.Ts)
            counter = counter+1;
            problem.yr{counter} = [ 0.5; 0; 0.5; 0; 0; 0; 0; 0 ];
            problem.ur{counter} = [ 0; 0 ];
        end
        for i = 18 : problem.Ts : (26-problem.Ts)
            counter = counter+1;
            problem.yr{counter} = [ 0.1; 0; 0.5; 0; 0; 0; 0; 0 ];
            problem.ur{counter} = [ 0; 0 ];
        end
        for i = 26 : problem.Ts : (35 + problem.Ts*problem.ni)
            counter = counter+1;
            problem.yr{counter} = [ 0.2; 0; 0.8; 0; 0; 0; 0; 0 ];
            problem.ur{counter} = [ 0; 0 ];
        end

        
    case 2
        % LTI simulation
        problem.simModel = [];
        counter = 0;
        for i = 0 : problem.Ts : (2-problem.Ts)
            counter = counter+1;
            problem.yr{counter} = [ 0; 0; 1; 0; 0; 0; 0; 0 ];
            problem.ur{counter} = [ 0; 0 ];
        end
        for i = 2 : problem.Ts : (10-problem.Ts)
            counter = counter+1;
            problem.yr{counter} = [ 0.2; 0; 0.7; 0; 0; 0; 0; 0 ];
            problem.ur{counter} = [ 0; 0 ];
        end
        for i = 10 : problem.Ts : (18-problem.Ts)
            counter = counter+1;
            problem.yr{counter} = [ 0.5; 0; 0.5; 0; 0; 0; 0; 0 ];
            problem.ur{counter} = [ 0; 0 ];
        end
        for i = 18 : problem.Ts : (26-problem.Ts)
            counter = counter+1;
            problem.yr{counter} = [ 0.1; 0; 0.5; 0; 0; 0; 0; 0 ];
            problem.ur{counter} = [ 0; 0 ];
        end
        for i = 26 : problem.Ts : (35 + problem.Ts*problem.ni)
            counter = counter+1;
            problem.yr{counter} = [ 0.2; 0; 0.8; 0; 0; 0; 0; 0 ];
            problem.ur{counter} = [ 0; 0 ];
        end
        
    otherwise
        error( 'Invalid variant number!' );
end

end
