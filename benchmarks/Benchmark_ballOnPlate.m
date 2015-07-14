function [ problem ] = Benchmark_ballOnPlate( variant )
%Benchmark_ballOnPlate does something.
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
problem = problem.loadFromMatFile( 'ballOnPlate',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.ballOnPlate);
problem.info.name         = 'ballOnPlate';
problem.info.description  = 'Single axis control of a ball balancing on a plate.';
problem.info.reference    = 'S. Richter: Computational Complexity Certification of Gradient Methods for Real-Time Model Predictive Control (sec. 9.6.1, p. 151), Dissertation, ETH Zurich, 2012.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.yes;


%% define MPC problem data
problem.Ts = 10e-3; % sampling time [s]

problem.A  = [1  0.01  ;
              0     1 ];
          
problem.B  = [ -0.0004  ;
               -0.0701 ];
           
problem.C  = eye(2);

problem.Q  = [100   0  ;
                0  10 ];
            
%problem.R = 0.1;
problem.R = 1;

problem.P = problem.Q;

problem.umin = -0.0524;
problem.umax = 0.0524;
problem.ymin = [-0.2; -0.1];  % y(1): ball position [m]
problem.ymax = [0.01;  0.1];  % y(2): ball velocity [m/s]

% Note that neither the input nor the state constraints reflect the maximum
% physical constraints of the original ball on plate system


%% define control scenario
problem.ni = 15;

for i = 1:5*problem(1).ni % regulate to the origin
    problem.yr{i} = [0;0.05];
end

problem.variants = [1 2 3 4];

switch ( variant )

    case 1
        % no active constraints in optimal solution
        problem.x0   = [-0.05; 0.05];
        
    case 2
        % some active constraints in optimal solution
        problem.x0   = [-0.05; 0];
        
    case 3
        % state constraints activated
        problem.x0 = [-0.17 0]';
        problem.umin = -0.03;
        
    case 4
        problem.ni = 25;
        % state constraints activated
        problem.x0 = [-0.15 0]';
        %problem.umin = -0.04;
        problem.yr = [];
        for i = 1:175 % regulate to the origin
            problem.yr{i} = [0;0];
        end
        
    otherwise
        
        error( 'Invalid variant number!' );
end

end

