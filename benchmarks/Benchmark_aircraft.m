function [ problem ] = Benchmark_aircraft( variant )
%Benchmark_aircraft does something.
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
problem = BenchmarkData(variant);
problem = Benchmark(problem);

% try to load benchmark from mat file
problem = problem.loadFromMatFile( 'aircraft',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.aircraft);
problem.info.name         = 'aircraft';
problem.info.description  = 'Control of an AFTI-F16 aircraft with saturating actuators.';
problem.info.reference    = 'http://ch.mathworks.com/help/mpc/examples/aircraft-with-unstable-poles.html';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.no;


%% define MPC benchmark data
Ac = [-0.0151   -60.5651   0       -32.174  ;
      -0.0001    -1.3411   0.9929    0      ;
       0.00018   43.2541  -0.86939   0      ;
       0          0        1         0     ];

Bc = [  -2.516   -13.136   ;
        -0.1689   -0.2514  ;
       -17.251    -1.5766  ;
         0         0      ];

if variant < 10
    
    Cc = [0  1  0  0  ;
          0  0  0  1 ];

    Dc = zeros(2,2);

else % variant >= 10: modified model so that y = x
    
    Cc = eye(4);

    Dc = zeros(4,2);
  
end

problem.Ts = 0.05; % Sampling time [s]

sysd = c2d(ss(Ac,Bc,Cc,Dc),problem.Ts);
[problem.A, problem.B, problem.C,problem.D] = ssdata(sysd);

problem.umax = [  25;   25];
problem.umin = [ -25;  -25];

if variant < 10
    problem.ymax = [ 0.5;  100];
    problem.ymin = [-0.5; -100]; % We add constraints on the outputs to limit undershoots
    
    problem.Q = 10*eye(2);

else
    problem.ymax = [ inf;  0.5;  inf;  100];
    problem.ymin = [-inf; -0.5; -inf; -100];
    
    problem.Q = 10*eye(4);

end

problem.R  = zeros(2);

problem.x0   = zeros(4,1);
problem.ni   = 10;
problem.uIdx = [1 2 3 6]; % Control horizon = 3


%% define control scenario
problem.variants = [1 2 3 10 11 12 13 14];

switch ( variant )
    
    case 1

        % original variant from reference
        for i = 1:problem(1).ni*5
            problem.yr{i} = [0;10];
        end
        
        
    case 2
        
        % same as original variant but with control horizon same as
        % prediction horizon. More computationally demanding. A small 
        % weight on inputs is added to avoid oscilations in solution
        
        for i = 1:problem(1).ni*7
            problem.yr{i} = [0;10];
        end
        problem.uIdx = [];
        problem.R    = 0.001*eye(2);  

        
    case 3
        
        % original variant from reference (but without state bounds
        for i = 1:problem(1).ni*5
            problem.yr{i} = [0;10];
        end
        problem.ymax = [];
        problem.ymin = [];

        
    case 4
        
        % original variant from reference but without move blocking
        problem.uIdx = [];
        for i = 1:problem(1).ni*5
            problem.yr{i} = [0;10];
        end

        
    case 10
        
        for i = 1:problem(1).ni*5
            problem.yr{i} = [0; 0; 0; 10];
        end
        % setting to zero the states that we do not care about so as to
        % have the equivalent problem with full state information.
        problem.Q(1,1) = 0;
        problem.Q(3,3) = 0;
        
        
    case 11
        
        problem.Q(1,1) = 0.1;       % to prevent 'instability' of state 1
        problem.Q(3,3) = 0;
        
        problem.uIdx = [];            % to make more computationally expensive
        problem.R    = 0.0001*eye(2); % to avoid oscilations
        
        for i = 1:problem(1).ni*5
            problem.yr{i} = [0; 0; 0; 10];  % TODO: probably delete and keep only variant 12
        end
        
        
    case 12
        
        % STANDFORD PAPER - NUMERICAL EXAMPLE
        
        problem.Q    = diag([10^-4 100 10^-3 100]);
        problem.R    = diag([10^-2 10^-2]);
        problem.P    = problem.Q;
        problem.uIdx = []; % not mentioned in paper
        
        for i = 1:35
            problem.yr{i} = [0; 0; 0; 10];
        end
        
        for i = 36:80
            problem.yr{i} = [0; 0; 0; 0];
        end
        
        
    case 13
        
        problem.Q    = diag([10^-4 100 10^-3 100]);
        problem.R    = diag([10^-2 10^-2]);
        problem.P    = problem.Q;
        problem.uIdx = [];
        
        for i = 1:50
            problem.yr{i} = [0; 0; 0; 10];
        end
        for i = 51:100
            problem.yr{i} = [0; 0; 0; 0];
        end
        for i = 101:150
            problem.yr{i} = [0; 0; 0; 5];
        end
        for i = 151:200
            problem.yr{i} = [0; 0; 0; -1];
        end
        for i = 201:250
            problem.yr{i} = [0; 0; 0; 0];
        end

        
    case 14
        
        problem.Q    = diag([10^-4 100 10^-3 100]);
        problem.R    = diag([10^-2 10^-2]);
        problem.P    = problem.Q;
        problem.uIdx = [];
        problem.ni   = 50;
        problem.lookAhead = Boolean.no;
        
        for i = 1:50
            problem.yr{i} = [0; 0; 0; 10];
        end
        for i = 51:100
            problem.yr{i} = [0; 0; 0; 0];
        end
        for i = 101:150
            problem.yr{i} = [0; 0; 0; 5];
        end
        for i = 151:200
            problem.yr{i} = [0; 0; 0; -1];
        end
        for i = 201:250
            problem.yr{i} = [0; 0; 0; 0];
        end

        
    otherwise
        error( 'Invalid variant number!' );
        
end

end
