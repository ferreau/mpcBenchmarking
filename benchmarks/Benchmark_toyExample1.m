function [ problem ] = Benchmark_toyExample1( variant )
%Benchmark_toyExample1 does something.
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
problem = problem.loadFromMatFile( 'toyExample1',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.toyExample1);
problem.info.name         = 'toyExample1';
problem.info.description  = 'A simple toy example.';
problem.info.reference    = 'Problem Set 6 - Numerical Optimization Methods, Model Predictive Control course at ETH Zurich in 2013';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.yes;


%% define MPC problem data
problem.A = [0.7115 -0.4345;
             0.4345  0.8853];
 
problem.B = [0.2173;
             0.0573];


% Constraints -------------------------------------------------------------
% umin <= u[k] <= umax , k = 0,1,...N-1
% xmin <= x[k] <= xmax , k = 0,1,...N
% Aineq*x <= bineq
% Aineq*u <= bineq

problem.umin = -5;
problem.umax = 5;
%problem.ymin = [-inf; -inf];
%problem.ymax = [ inf;  inf];

% Default objective function ----------------------------------------------
% Suggested values - Can be edited on main script for further investigation 
%                N
%       min   SUM  { x[k]'*Q*x[k] + u[k]'*R*u[k] + ...} + x[N]'*P*x[N]
%  x[k],u[k]   k=0

problem.Q  = 10*eye(2);  % Q matrix, symmetric, positive semi-definite
problem.R  = 1;          % R matrix, symmetric, postive  definite (*want to penalize the control in all directions)
problem.ni = 10;         % N horizon length


[~,problem.P] = dlqr(problem.A,problem.B, ... 
    problem.Q,problem.R);


%% define control scenario
problem.variants = [1 2 3 4 5];

switch variant
        
    case 1
        
        problem.x0 = [10;0];
        
    case 2
        
        problem.x0 = [10;0];
        problem.ni = 20;
        for i = 1:problem.ni+21
            problem.yr{i} = [0;0];
        end
        for i = problem.ni+21:problem.ni+100
            problem.yr{i} = [5;15];
        end
        for i = problem.ni+100:problem.ni+200
            problem.yr{i} = [-5;-5];
        end

    case 3
        problem.x0   = [100; -100];
        problem.ni   = 10;
        problem.uIdx = [1 2];
        for i = 1:problem.ni+50
            problem.yr{i} = [0;0];
        end
        
    case 4
        problem.x0 = [100; -100];
        problem.ni = 50;
        for i = 1:problem.ni + 50
            problem.yr{i} = [0;0];
        end
        
    case 5
        problem.x0 = [100; -100];
        problem.ni = 100;
        for i = 1:problem.ni+25
            problem.yr{i} = [0;0];
        end
        for i = problem.ni+ 25 + 1 : problem.ni + 50
            problem.yr{i} = [10;10];
        end

    otherwise
        error( 'Invalid variant number!' );
end

end
