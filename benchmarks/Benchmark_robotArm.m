function [ problem ] = Benchmark_robotArm( variant )
%Benchmark_robotArm does something.
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
% Last modified:   30/11/2015


% default variant
if ( nargin < 1 )
    variant = 1;
end


%% define MPC problem info
problem = Benchmark(variant); % create an empty benchmark object

% try to load benchmark from mat file
problem = problem.loadFromMatFile( 'robotArm',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.robotArm);
problem.info.name         = 'robotArm';
problem.info.description  = 'Two-link robot arm build with the Lego NXT platform.';
problem.info.reference    = 'P. Zometa, M. Kögel, R. Findeisen: muAO-MPC: a Free Code Generation Tool for Embedded Real-Time Linear Model Predictive Control. ACC, Washington, USA, pp. 5320-5325, 2013.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.allFeasible;
problem.info.isOpenLoopStable = Boolean.yes;


%% define MPC benchmark data
problem.Ts = 0.004; % [s]

Ac = [  0.00e+0,  1.00e+0,  0.00e+0,  0.00e+0; ...
        0.00e+0, -1.72e+1,  0.00e+0,  0.00e+0; ...
        0.00e+0,  0.00e+0,  0.00e+0,  1.00e+0; ...
        0.00e+0,  1.00e+0,  0.00e+0, -1.61e+1  ];

Bc = [  0.00e+0,  0.00e+0; ...
        2.62e+0,  0.00e+0; ...
        0.00e+0,  0.00e+0; ...
        0.00e+0,  2.48e+0  ];
    
Cc = eye(4);

sys = c2d( ss(Ac,Bc,Cc,[]),problem.Ts,'zoh' );
[A,B,C,D] = ssdata(sys);


problem.A = A;        % compulsory
problem.B = B;        % compulsory
problem.C = C;        % optional (default, identity matrix)
problem.D = D;        % optional (default, zero matrix)

problem.umax =  [ 100; 25 ];            % all constraint fields are optional
problem.umin = -[ 100; 25 ];            % (default values -inf/inf)

problem.ymax =  [ Inf; 1; Inf; 1 ];     % all constraint fields are optional
problem.ymin = -[ Inf; 1; Inf; 1 ];     % (default values -inf/inf)

problem.Q  = diag([ 1.14e+4, 2.24e+1, 1.14e+4, 2.94e+1 ]);  % 
problem.R  = diag([ 2.20e-1, 2.37e-1 ]);                    % 
problem.P = problem.Q;
         
problem.x0 = [ -1; 0; 1; 0 ];

problem.uIdx = [];                  % optional (default, []. Control horizon same as prediction horizon)
problem.lookAhead = Boolean.yes;    % optional (default, 0.  No access to future value of reference trajectories)
problem.simModel  = [];             % optional (default, simulation model same as prediction model)


%% define control scenario
problem.variants = [1 2];

switch ( variant )
    
    case 1
        % original formulation as in reference
        problem.ni = 5;
        for i=1:problem.ni+400
            problem.yr{i} = zeros( 4,1 );
        end
        
    case 2
        % longer control horizon to improve tracking and to
        % make more constraints active
        problem.ni = 20;
        for i=1:problem.ni+400
            problem.yr{i} = zeros( 4,1 );
        end


    otherwise
        error( 'Invalid variant number!' );
end

end
