function [ problem ] = Benchmark_template( variant )
%Benchmark_template is NOT intended to be run, but to provide a template
%for adding new benchmark examples.
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
problem = benchmark( variant ); % create an empty benchmark object

% try to load benchmark from mat file
problem = problem.loadFromMatFile( 'template',variant );

problem.info              = setupBenchmarkInfoStruct( );
%problem.info.ID           = uint32(Benchmarks.template);
problem.info.name         = 'template';
problem.info.description  = 'describe new benchmark here';
problem.info.reference    = 'add a citation here';
problem.info.origin       = Origin.undefined;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.yes;


%% define MPC benchmark data

% example of a simple LTI system

problem.A = [1 1; 0 1];        % mandatory
problem.B = [1  ; 0.5];        % mandatory
problem.f = [0  ; 0  ];        % optional (default, zero vector)
problem.C = eye(2);            % optional (default, identity matrix)
problem.D = zeros(2,1);        % optional (default, zero matrix)
problem.e = [0  ; 0  ];        % optional (default, zero vector)

problem.umax =  1;             % all constraint fields are optional
problem.umin = -1;             % (default values -inf/inf)
problem.ymax = [ 2;  5];
problem.ymin = [-2; -5];       % same for dmin,dmax,M,N,dNmin,dNmax,T

problem.Q  = eye(2);           % optional (default, identity matrix)
problem.R  = 0.1;              % optional (default, zero matrix)
problem.S  = zeros(2,1);       % optional (default, zero matrix)
problem.gu = 0;                % optional (default, zero vector)
problem.gy = zeros(2,1);       % optional (default, zero vector)

[~, problem.P] = dlqr(problem.A, problem.B, problem.Q, ...
    problem.R, problem.S);     % optional (default, zero matrix)


problem.yr  = zeros(2,1);      % optional (default, zero vector)
problem.ur  = 0;               % optional (default, zero vector)
problem.xNr = zeros(2,1);      % optional (default, zero vector)


problem.x0 = 10*ones(2,1);     % compulsory      

problem.ni   = 15;              % compulsory      
problem.uIdx = [];              % optional (default, []. Control horizon same as prediction horizon)
problem.lookAhead = Boolean.no; % optional (default, 0.  No access to future value of reference trajectories)
problem.simModel  = 0;          % optional (default, 0.  Simulation model same as prediction model)


%% define control scenario
problem.variants = [1 2];

switch ( variant )

    case 1
        
        % variant 1
        
    case 2
        
        % variant 2
        problem.x0 = rand(2,15); % 15 random initial conditions
      
    otherwise
        error( 'Invalid variant number!' );
        
end

end
