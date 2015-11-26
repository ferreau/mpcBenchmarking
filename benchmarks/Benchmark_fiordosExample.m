function [ problem ] = Benchmark_fiordosExample( variant )
%Benchmark_fiordosExample does something.
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
problem = problem.loadFromMatFile( 'forcesExample',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.fiordosExample);
problem.info.name         = 'fiordosExample';
problem.info.description  = 'Toy example for input-only and input/state constrained linear MPC with terminal weight.';
problem.info.reference    = 'FiOrdOs webpage: http://fiordos.ethz.ch/dokuwiki/doku.php?id=mpcexample, 2012.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.allFeasible;
problem.info.isOpenLoopStable = Boolean.no;


%% define MPC problem data
problem.Ts = 1; % [s]

problem.A = [0.8 1; 0 0.9]; 
problem.B = [-1; 2]; 
problem.C = eye(2);
problem.D = [0; 0];

% Constraints -------------------------------------------------------------
problem.umax =  0.7;
problem.umin = -0.7;

% Default objective function ----------------------------------------------
problem.Q  = eye(2);   
problem.R  = 1;          
problem.ni = 5;

%% define control scenario
problem.variants = [1 2 3];

switch ( variant )
    
    case 1 % original input-constrained example
        problem.x0 = [1; 2];
        problem.P = dlyap( problem.A',problem.Q );
        for i = 1:problem.ni+20
            problem.yr{i} = [0;0];
        end  
                
    case 2 % original input and state-constrained example
        problem.x0 = [1; 2];
        problem.ymax = [ 3.5;  2  ];
        problem.ymin = [-3.5; -0.5];
        [~,problem.P] = dlqr( problem.A,problem.B,problem.Q,problem.R );
        for i = 1:problem.ni+20
            problem.yr{i} = [0;0];
        end
        
    case 3 % input-constrained example with more challenging initial state
        problem.x0 = [10; 20];
        problem.P = dlyap( problem.A',problem.Q );
        for i = 1:problem.ni+20
            problem.yr{i} = [0;0];
        end  

    otherwise
        error( 'Invalid variant number!' );
        
end
