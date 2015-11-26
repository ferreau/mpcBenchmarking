function [ problem ] = Benchmark_doubleInvertedPendulum( variant )
%Benchmark_doubleInvertedPendulum does something.
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
% Last modified:   24/11/2015


% default variant
if ( nargin < 1 )
    variant = 1;
end


%% define MPC problem info
problem = Benchmark(variant); % create an empty benchmark object

% try to load benchmark from mat file
problem = problem.loadFromMatFile( 'doubleInvertedPendulum',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.doubleInvertedPendulum);
problem.info.name         = 'doubleInvertedPendulum';
problem.info.description  = 'Linearized double inverted pendulum. Unstable and possibly with state constraints.';
problem.info.reference    = 'E.J. Davison: Benchmark Problems for Control System Design, IFAC, 1990.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.no;


%% define MPC benchmark data
problem.Ts = 0.01; % [s]

Ac = [  0.00e+0,  1.00e+0,  0.00e+0,  0.00e+0; ...
        9.80e+0,  0.00e+0, -9.80e+0,  0.00e+0; ...
        0.00e+0,  0.00e+0,  0.00e+0,  1.00e+0; ...
       -9.80e+0,  0.00e+0,  2.94e+1,  0.00e+0  ];

Bc = [  0.00e+0,  0.00e+0; ...
        1.00e+0, -2.00e+0; ...
        0.00e+0,  0.00e+0; ...
       -2.00e+0,  5.00e+0  ];
    
Cc = [ 1, 0, 0, 0; ...
       0, 0, 1, 0  ];

sys = c2d( ss(Ac,Bc,Cc,[]),problem.Ts,'zoh' );
[A,B,C,D] = ssdata(sys);


problem.A = A;        % compulsory
problem.B = B;        % compulsory
problem.C = C;        % optional (default, identity matrix)
problem.D = D;        % optional (default, zero matrix)

problem.umax =  [5.0; 5.0];             % all constraint fields are optional
problem.umin = -[5.0; 5.0];             % (default values -inf/inf)

problem.Q  = 100*eye(2);           % optional (default, identity matrix)
problem.R  = 0.1*eye(2);              % optional (default, zero matrix)

[~, problem.P] = dlqr( problem.A, problem.B, problem.C'*problem.Q*problem.C,problem.R, zeros(4,2) );

problem.ni   = 10;                  %       
problem.uIdx = [];                  % optional (default, []. Control horizon same as prediction horizon)
problem.lookAhead = Boolean.yes;    % optional (default, 0.  No access to future value of reference trajectories)
problem.simModel  = [];             % optional (default, simulation model same as prediction model)


%% define control scenario
problem.variants = [1 2 3];

switch ( variant )
    
    case 1
       
        problem.x0 = [ 0.1, 0.05, 0.1, 0.05 ]';
        for i=1:450
            problem.yr{i} = [0;0];
        end

        
    case 2
       
        problem.x0 = [ 0.0, 0.0, 0.1, 0.5 ]';
        for i=1:450
            problem.yr{i} = [0;0];
        end

        
    case 3
       
        problem.x0 = [ 0.1, 0.05, 0.1, 0.05 ]';
        
        problem.ymax =  [0.12; 0.12];             % all constraint fields are optional
        problem.ymin = -[0.12; 0.12];             % (default values -inf/inf)
        
        for i=1:450
            problem.yr{i} = [0;0];
        end

        
    otherwise
        error( 'Invalid variant number!' );
end

end
