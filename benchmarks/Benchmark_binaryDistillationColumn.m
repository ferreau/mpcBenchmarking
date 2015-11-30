function [ problem ] = Benchmark_binaryDistillationColumn( variant )
%Benchmark_binaryDistillationColumn does something.
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
problem = problem.loadFromMatFile( 'binaryDistillationColumn',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.binaryDistillationColumn);
problem.info.name         = 'binaryDistillationColumn';
problem.info.description  = 'Fairly realistic model of a binary distillation column.';
problem.info.reference    = 'E.J. Davison: Control of a distillation column with pressure variation, Trans. Institute of Chemical Engineers, vol. 45, pp. 229-250, 1967.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.yes;


%% define MPC benchmark data
problem.Ts = 60; % [s]

Ac = [ -1.40e-2,  4.30e-3,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0; ...
        9.50e-3, -1.38e-2,  4.60e-3,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  5.00e-4; ...
        0.00e+0,  9.50e-3, -1.41e-2,  6.30e-3,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  2.00e-4; ...
        0.00e+0,  0.00e+0,  9.50e-3, -1.58e-2,  1.10e-2,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0; ...
        0.00e+0,  0.00e+0,  0.00e+0,  9.50e-3, -3.12e-2,  1.50e-2,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0; ...
        0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  2.02e-2, -3.52e-2,  2.20e-2,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0; ...
        0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  2.02e-2, -4.22e-2,  2.80e-2,  0.00e+0,  0.00e+0,  0.00e+0; ...
        0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  2.02e-2, -4.82e-2,  3.70e-2,  0.00e+0,  2.00e-4; ...
        0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  2.02e-2, -5.72e-2,  4.20e-2,  5.00e-4; ...
        0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  2.02e-2, -4.83e-2,  5.00e-4; ...
        0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  0.00e+0,  2.55e-2, -1.85e-2  ];

Bc = [  0.00e+0,  0.00e+0,  0.00e+0; ...  
        5.00e-6, -4.00e-5,  2.50e-3; ...  
        2.00e-6, -2.00e-5,  5.00e-3; ...
        1.00e-6, -1.00e-5,  5.00e-3; ...
        0.00e+0,  0.00e+0,  5.00e-3; ...
        0.00e+0,  0.00e+0,  5.00e-3; ...
       -5.00e-6,  1.00e-5,  5.00e-3; ...
       -1.00e-5,  3.00e-5,  5.00e-3; ...
       -4.00e-5,  5.00e-6,  2.50e-3; ...
       -2.00e-5,  2.00e-6,  2.50e-3; ...
        4.60e-4,  4.60e-4,  0.00e+0  ];
    
Cc = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0; ...
       1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ...
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1  ];

sys = c2d( ss(Ac,Bc,Cc,[]),problem.Ts,'zoh' );
[A,B,C,D] = ssdata(sys);


problem.A = A;        % compulsory
problem.B = B;        % compulsory
problem.C = C;        % optional (default, identity matrix)
problem.D = D;        % optional (default, zero matrix)

problem.umax =  [2.5; 2.5; 0.3];             % all constraint fields are optional
problem.umin = -[2.5; 2.5; 0.3];             % (default values -inf/inf)

problem.Q  = 100*eye(3);           % optional (default, identity matrix)
problem.R  = 1*eye(3);              % optional (default, zero matrix)

%{
problem.deltaInput = setupDeltaInputStruct();
problem.deltaInput.active = Boolean.yes;
problem.deltaInput.weight = problem.R;
problem.deltaInput.dumax =  [0.05; 0.05; 0.05];
problem.deltaInput.dumin = -[0.05; 0.05; 0.05];
%}


% note: arbitrary steady-state computed without any process insight!
problem.x0 = [ 3.3804e-01; ...
               1.1006e+00; ...
               2.4606e+00; ...
               3.7428e+00; ...
               3.2063e+00; ...
               4.2654e+00; ...
               3.8579e+00; ...
               2.7192e+00; ...
               1.4173e+00; ...
               6.0670e-01; ...
               8.8599e-01  ];

problem.ni   = 15;                  % 
problem.uIdx = [];                  % optional (default, []. Control horizon same as prediction horizon)
problem.lookAhead = Boolean.yes;    % optional (default, 0.  No access to future value of reference trajectories)
problem.simModel  = [];             % optional (default, simulation model same as prediction model)


%% define control scenario
problem.variants = [1 2];

switch ( variant )
    
    case 1
        for i=1:problem.ni+50
            problem.yr{i} = [0;0;0];
        end
        
    case 2
        problem.uIdx = [1:5];
        for i=1:problem.ni+50
            problem.yr{i} = [0;0;0];
        end

    otherwise
        error( 'Invalid variant number!' );
end

end
