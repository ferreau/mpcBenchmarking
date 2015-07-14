function [ problem ] = Benchmark_pendulum( variant )
%Benchmark_pendulum does something.
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
problem = Benchmark(variant); % create an empty benchmark object

% try to load benchmark from mat file
problem = problem.loadFromMatFile( 'pendulum',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.pendulum);
problem.info.name         = 'pendulum';
problem.info.description  = 'Inverted pendulum example.';
problem.info.reference    = 'U. Maeder, R. Cagienard, M. Morari: Explicit Model Predictive Control, in: Lecture Notes in Control and Information Sciences (LNCIS), vol 346: Advanced Strategies in Control Systems with Input and Output Constraints, pp. 237--271, Springer, 2007';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.no;


%% define MPC benchmark data

% example of a simple LTI system
problem.A = [   1.001      -0.05       -0.001;...
               -0.05        1.003       0.05;...
               -0.001       0.05        1.001]; 
           
problem.B = [0  ; 0.001; 0.05];        % compulsory

problem.umax =  1.25;             % all constraint fields are optional
problem.umin = -1.25;             % (default values -inf/inf)


problem.Q  = eye(3);           % optional (default, identity matrix)
problem.R  = 0.1;              % optional (default, zero matrix)
problem.S  = zeros(3,1);       % optional (default, zero matrix)


[~, problem.P] = dlqr(problem.A, problem.B, problem.Q, ...
    problem.R, problem.S);     % optional (default, zero matrix)

%problem.P = diag(diag(problem.P));


% problem.yr  = [0;0;0];      % optional (default, zero vector)
problem.ur  = 0;               % optional (default, zero vector)

problem.x0 = 0.6*ones(3,1);    % compulsory      

problem.ni   = 15;              % compulsory      
problem.uIdx = [];              % optional (default, []. Control horizon same as prediction horizon)
problem.lookAhead = Boolean.no; % optional (default, 0.  No access to future value of reference trajectories)
problem.simModel  = [];         % optional (default, simulation model same as prediction model)


%% define control scenario
problem.variants = [1 2 3];

switch ( variant )
    
    case 1

        for i=1:problem.ni+150
            problem.yr{i} = [0;0;0];
        end
        
    case 2
        
        for i=1:problem.ni+150
            problem.yr{i} = [0;0;0];
        end
        % to preserve diagonal structure for fiordos without loosing
        % stability properties
        problem.P = diag([problem.P(1,1) problem.P(2,2) problem.P(3,3)]);
        
    case 3
        % ???
        problem.ni = 2;  
        for i=1:problem.ni+300
            problem.yr{i} = [0;0;0];
        end

      
    otherwise
        error( 'Invalid variant number!' );
end

end
