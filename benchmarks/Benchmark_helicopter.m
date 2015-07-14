function [ problem ] = Benchmark_helicopter( variant )
%Benchmark_helicopter does something.
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
problem = problem.loadFromMatFile( 'helicopter',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.helicopter);
problem.info.name         = 'helicopter';
problem.info.description  = 'Laboratory model of a Quansar 3-DOF helicopter. Note that dynamics have been changed to make system stabilizable.';
problem.info.reference    = 'P. Tondel, T.A. Johansen: Complexity Reduction in Explicit Linear Model Predictive Control, 15th IFAC World Congress, 2002.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.yes;


%% define MPC benchmark data

% example of a simple LTI system

% problem.A = [   1       0       0.01    0       0       0;...
%                 0       1       0       0.01    0       0;...
%                 0       0       1       0       0       0;...
%                 0       0       0       1       0       0;...
%                 0.01    0       0       0       1       0;...
%                 0       0.01    0       0       0       1];
            
problem.A = [   0.99       0       0.01    0       0       0;...
                0       0.99       0       0.01    0       0;...
                0       0       0.99       0       0       0;...
                0       0       0       0.99       0       0;...
                0.01    0       0       0       0.99       0;...
                0       0.01    0       0       0       0.99];


problem.B = [   0           0;...
                0.0001      -0.0001;...
                0.0019      -0.0019;...
                0.0132      -0.0132;...
                0           0;...
                0           0];
            
% problem.B = [   0.001       -0.001;...
%                 0.0001      -0.0001;...
%                 0.0019      -0.0019;...
%                 0.0132      -0.0132;...
%                 0.01       -0.01;...
%                 0.01       -0.01];
            
problem.umax =  [   3;     3    ];             
problem.umin =  [  -1;    -1    ];             
problem.ymax =  [   inf;    inf;    0.44;   0.6;    inf;    inf];
problem.ymin = -[   inf;    inf;    0.44;   0.6;    inf;    inf];

problem.Q  = eye(6);           
problem.Q(1,1) = 100;
problem.Q(2,2) = 100;
problem.Q(3,3) = 10;
problem.Q(4,4) = 10;
problem.Q(5,5) = 400;
problem.Q(6,6) = 200;
problem.R  = 0.001*eye(2);
problem.S = zeros(6,2);

problem.T = eye(6);
problem.dNmin = -ones(6,1);
problem.dNmax = ones(6,1);

[~, problem.P] = dlqr(problem.A, problem.B, problem.Q, ...
    problem.R, problem.S);     % optional (default, zero matrix)

problem.x0 = [0.5,0.5,0,0,0,0]' ;    % compulsory      

problem.ni   = 3;                 % compulsory      
problem.uIdx = [];                % optional (default, []. Control horizon same as prediction horizon)
problem.lookAhead = Boolean.no;   % optional (default, 0.  No access to future value of reference trajectories)
problem.simModel = [];            % optional (default, simulation model same as prediction model)


%% define control scenario
problem.variants = [1 2 3 4 5];

switch ( variant )
    
    case 1
        
        for i=1:problem.ni+150
            problem.yr{i} = [0;0;0;0;0;0];
        end        
        

    case 2
        
        problem.ni = 20;
        for i=1:problem.ni+150
            problem.yr{i} = [0;0;0;0;0;0];
        end
        

    case 4
        
        problem.ni = 10;
        for i=1:problem.ni+50
            problem.yr{i} = [0;0;0;0;0;0];
        end
        [~, problem.P] = dlqr(problem.A, problem.B, problem.Q, ...
            problem.R, problem.S);     % optional (default, zero matrix)
        
        problem.P = diag(diag(problem.P));
        problem.R  = 1*eye(2);


    case 5
        
        % variant 5
        problem.x0 = rand(2,15); % 15 random initial conditions
      

    otherwise
        error( 'Invalid variant number!' );
end

end
