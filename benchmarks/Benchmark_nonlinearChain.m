function [ problem ] = Benchmark_nonlinearChain( variant )
%Benchmark_nonlinearChain does something.
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
problem = problem.loadFromMatFile( 'nonlinearChain',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.nonlinearChain);
problem.info.name         = 'nonlinearChain';
problem.info.description  = 'Chain of ten masses connected by undamped Hookian springs; one end is fixed to a wall, while the other end can be moved.';
problem.info.reference    = 'H.J. Ferreau: An Online Active Set Strategy for Fast Solution of Parametric Quadratic Programs with Applications to Predictive Engine Control. Diplom thesis, University of Heidelberg, 2006.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.allFeasible;


%% define MPC problem data

problem.Ts = 0.2; % [s] 

% settings for linearizing ODE model
fcnName = 'chainOde';
NX = 57;
NU = 3;
load nonlinearChain_ss.mat;
t0 = 0;
tEnd = problem.Ts;

[ problem.A,problem.B,problem.f ] = linearizeOde( fcnName,NX,NU, xSS,uSS,t0,tEnd, 'ode45',1e-8 );

problem.C  = eye( NX );
problem.D  = zeros( NX,NU );
    
problem.umax =  ones(NU,1);
problem.umin = -ones(NU,1);
problem.ymax =  Inf * ones(NX,1);
problem.ymin = -Inf * ones(NX,1);

alpha = 50;
beta  = 2;
gamma = 0.02;
xiWall = -0.2;

problem.Q = diag( [ zeros(9*3,1); alpha*ones(3,1); beta*ones(9*3,1) ] );
problem.R = diag( gamma*ones(3,1) );

problem.x0 = x0;


%% define control scenario
problem.variants = [1 2 3 4 11 12 13 14];

switch variant
    
    case 1
        % nonlinear simulation, without wall, long horizon
        problem.simModel = fcnName;
        problem.ni = 80;
        for i = 1:problem.ni+75
            problem.yr{i} = xSS;
            problem.ur{i} = uSS;
        end
        
    case 2
        % nonlinear simulation, with wall, long horizon
        problem.simModel = fcnName;
        problem.ni = 80;
        for i = 1:problem.ni+75
            problem.yr{i} = xSS;
            problem.ur{i} = uSS;
        end
        problem.ymax = Inf * ones(NX,1);
        problem.ymin = [ repmat( [-Inf;xiWall;-Inf],9,1 ); [-Inf;-Inf;-Inf]; -Inf*ones(9*3,1) ];

        
    case 3
        % nonlinear simulation, without wall, short horizon
        problem.simModel = fcnName;
        problem.ni = 20;
        for i = 1:problem.ni+75
            problem.yr{i} = xSS;
            problem.ur{i} = uSS;
        end
        
    case 4
        % nonlinear simulation, with wall, short horizon
        problem.simModel = fcnName;
        problem.ni = 20;
        for i = 1:problem.ni+75
            problem.yr{i} = xSS;
            problem.ur{i} = uSS;
        end
        problem.ymax = Inf * ones(NX,1);
        problem.ymin = [ repmat( [-Inf;xiWall;-Inf],9,1 ); [-Inf;-Inf;-Inf]; -Inf*ones(9*3,1) ];
        
        
    case 11
        % LTI simulation, without wall, long horizon
        problem.simModel = [];
        problem.ni = 80;
        for i = 1:problem.ni+75
            problem.yr{i} = xSS;
            problem.ur{i} = uSS;
        end
        
    case 12
        % LTI simulation, with wall, long horizon
        problem.simModel = [];
        problem.ni = 80;
        for i = 1:problem.ni+75
            problem.yr{i} = xSS;
            problem.ur{i} = uSS;
        end
        problem.ymax = Inf * ones(NX,1);
        problem.ymin = [ repmat( [-Inf;xiWall;-Inf],9,1 ); [-Inf;-Inf;-Inf]; -Inf*ones(9*3,1) ];
        
    case 13
        % LTI simulation, without wall, short horizon
        problem.simModel = [];
        problem.ni = 20;
        for i = 1:problem.ni+75
            problem.yr{i} = xSS;
            problem.ur{i} = uSS;
        end
        
    case 14
        % LTI simulation, with wall, short horizon
        problem.simModel = [];
        problem.ni = 20;
        for i = 1:problem.ni+75
            problem.yr{i} = xSS;
            problem.ur{i} = uSS;
        end
        problem.ymax = Inf * ones(NX,1);
        problem.ymin = [ repmat( [-Inf;xiWall;-Inf],9,1 ); [-Inf;-Inf;-Inf]; -Inf*ones(9*3,1) ];

    otherwise
        error( 'Invalid variant number!' );
end

end

% REFERENCE: 
% ...
