function [ problem ] = Benchmark_polytopicTerminal( variant )
%Benchmark_polytopicTerminal does something.
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
% Last modified:   26/11/2015


% default variant
if ( nargin < 1 )
    variant = 1;
end


%% define MPC problem info
problem = Benchmark(variant);

% try to load benchmark from mat file
problem = problem.loadFromMatFile( 'polytopicTerminal',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.polytopicTerminal);
problem.info.name         = 'polytopicTerminal';
problem.info.description  = 'Simple SISO system with polyhedral terminal constraints.';
problem.info.reference    = 'A. Bemporad, D. Bernardini, P. Patrinos: A Convex Feasibility Approach to Anytime Model Predictive Control. http://arxiv.org/abs/1502.07974, 2015.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.allFeasible;


%% define MPC benchmark data
r = 0.5;

problem.A = [  0.4424  1.0000  ;
              -0.4746  0.4424  ];

problem.B = [  0.0;
               2.0623 ];
   
problem.C = [ -0.7013  1.9407 ];

problem.D = 0;

problem.Ts = 0.2; % Sampling time [s]

problem.umax =  1;
problem.umin = -1;

problem.ymax =  1;
problem.ymin = -1;

problem.Q = 10;
problem.R =  1;

[ ~,problem.P ] = dlqr( problem.A,problem.B, problem.C'*problem.Q*problem.C,problem.R );

K = 0.1 * [ 1 1 1 1 1 1 ]';
xr = [ 2.6252; 1.4639 ]*r;
ur = r;

problem.T = [ 1 0 -1  0  0.4424 -0.4424; 
              0 1  0 -1  1      -1       ]';
problem.dNmax = K + problem.T*xr;

problem.x0 = [ 0; 0];


%% define control scenario
problem.variants = [1 2];

switch ( variant )
    
    case 1
        % original variant from reference
        problem.ni = 6;
        for i = 1:problem.ni+50
            problem.yr{i} = r;
            problem.ur{i} = ur;
        end       
        
    case 2
        % alternative original variant with very long horizon
        problem.ni = 200;
        for i = 1:problem.ni+50
            problem.yr{i} = r;
            problem.ur{i} = ur;
        end
        
    otherwise
        error( 'Invalid variant number!' );
        
end

end
