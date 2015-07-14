function [ problem ] = Benchmark_quadcopter( variant )
%Benchmark_quadcopter does something.
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
problem = problem.loadFromMatFile( 'quadcopter',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.quadcopter);
problem.info.name         = 'quadcopter';
problem.info.description  = 'Position and velocity control of a quadcopter. Linearized model of quadcopter around steady state. 4 inputs, 12 outputs both with box constraints.';
problem.info.reference    = 'MPT toolbox 3.0 demos, see http://control.ee.ethz.ch/~mpt/3/Main/BenchmarkExamples';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.yes;


%% define MPC problem data
load quadcopter_ssmodel

% the model is linearized around the following steady state (added u0 to
% all inputs in the plots to have same results as in MPT demo)
u0 = [10.5916; 10.5916;  10.5916; 10.5916];
problem.x0 = zeros(12,1);
%problem.x0 = 0.15*[ pi/6;  pi/6;  5;  5;  5; 5; 5*ones(6,1)];

% system matrices
[problem.A, problem.B, problem.C, problem.D] = ssdata(sysd);

problem.umin = [9.6; 9.6; 9.6; 9.6] - u0;
problem.umax = [13; 13; 13; 13] - u0;

problem.ymin = [-pi/6; -pi/6; -Inf; -Inf; -Inf; -1; -Inf(6,1)];
problem.ymax = [ pi/6;  pi/6;  Inf;  Inf;  Inf; Inf; Inf(6,1)];

problem.Q = diag([0 0 10 10 10 10 0 0 0 5 5 5]);
problem.R = 0.1*eye(4);

problem.Ts   = 0.1; % samping time [s]
problem.ni   = 10;
problem.uIdx = [];


%% define control scenario
problem.variants = [1 2 3 4 5 6];

switch variant

    case 1
        
		problem.uIdx = [1 2];
        for i=1:problem.ni+30
            problem.yr{i} = [0;0;1;0;0;0;zeros(6,1)];
        end

        
	case 2

        problem.ni = 20;
        for i=1:problem.ni+50
            problem.yr{i} = [0;0;0;0;0;0;zeros(6,1)];
        end

        
    case 3

        problem.ni = 100;
        for i=1:problem.ni+50
            problem.yr{i} = [0;0;0;0;0;0;zeros(6,1)];
        end
        
        
    case 4 % relax input constraints and penalize more inputs: active output constraints 

        problem.Q = 0.0001*problem.Q;
        problem.R = 1000*problem.R;
        
        problem.x0 = 0.3*[ pi/6;  pi/6;  5;  5;  5; 5; 5*ones(6,1)];
        problem.umin = [9.6; 9.6; 9.6; 9.6] - u0 - 5*ones(4,1);
        problem.umax = [13; 13; 13; 13] - u0 + 5*ones(4,1);
        
        problem.ymin = [-pi/6; -pi/6; -Inf; -Inf; -Inf; -1; -Inf(6,1)];
        problem.ymax = [ pi/6;  pi/6;  Inf;  Inf;  Inf; Inf; Inf(6,1)];
        problem.ni = 20;
        for i=1:problem.ni+150
            problem.yr{i} = [0;0;0;0;0;0;zeros(6,1)];
        end

        
    case 5  % polytopic contraints: the quadrocopter has to lie in a
            % cylinder having for basis a regular poligon (++check consistency wrt reality)
            problem.x0 = 0.2*[ pi/6;  pi/6;  5;  8;  8; 5; 10*ones(6,1)];
        r = 3.5;
        problem.umin = [9.6; 9.6; 9.6; 9.6] - u0 - 5*ones(4,1);
        problem.umax = [13; 13; 13; 13] - u0 + 5*ones(4,1);
        problem.umin = [];
        problem.umax = [];
        problem.Q = 0.0001*problem.Q;
        problem.R = 1000*problem.R;
        problem.ni = 20;
        for i=1:problem.ni+150
            problem.yr{i} = [0;0;0;0;0;0;zeros(6,1)];
        end
        problem.ymin = [];
        problem.ymax = [];
        problem.M = [   0   0   0   -1   1   0   0   0   0   0   0   0;  ...
                        0   0   0   1   1   0   0   0   0   0   0   0;  ...
                        0   0   0  1   1   0   0   0   0   0   0   0;  ...
                        0   0   0  -1   1   0   0   0   0   0   0   0;    ];
                        
        problem.N = zeros(4,4);
        problem.dmin = [-Inf; -Inf;-r;-r];
        problem.dmax = [r; r;Inf; Inf];
        

    case 6 % fiordos

        [~, problem.P] = dlqr(problem.A, problem.B, problem.Q, ...
        problem.R, problem.S);     % optional (default, zero matrix)
        problem.P = diag(diag(problem.P));
        problem.x0 = 0.15*[ pi/6;  pi/6;  5;  5;  5; 5; ones(6,1)];
        problem.Q = diag([0.1 0.1 10 10 10 10 0.01 0.1 0.1 5 5 5]);
        problem.R = 0.1*eye(4);
        problem.ni = 10;
        for i=1:problem.ni+50
            problem.yr{i} = [0;0;0;0;0;0;zeros(6,1)];
        end


	otherwise       
        error( 'Invalid variant number!' );

end

end

