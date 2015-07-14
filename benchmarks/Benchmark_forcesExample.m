function [ problem ] = Benchmark_forcesExample( variant )
%Benchmark_forcesExample does something.
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
problem.info.ID           = uint32(Benchmarks.forcesExample);
problem.info.name         = 'forcesExample';
problem.info.description  = 'Simple MPC example';
problem.info.reference    = 'http://forces.ethz.ch/doku.php?id=examples:simplempc';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.no;


%% define MPC problem data
problem.A = [1.1 1; 0 1];
problem.B = [1;0.5];

problem.umin = -0.5;
problem.umax =  0.5;

problem.Q = eye(2);
problem.R = 1;

[~,problem.P] = dlqr(problem.A,problem.B,problem.Q,problem.R);
%problem.P = diag(diag(problem.P));

problem.ymin = [-5 ; -5];
problem.ymax = [ 5 ;  5];

problem.ni = 9;             % instead of 10 in the website due to different index convention on problem formulation
problem.x0 = [-4; 2];


%% define control scenario
problem.variants = [1 2 3 4];

switch variant
        
    case 1
    
        for i = 1:problem.ni+21
            
            problem.yr{i} = [0;0];
        end

    case 2
        problem.ni = 20;
        for i = 1:problem.ni+25
            problem.yr{i} = [0;0];
        end
        for i = problem.ni+26:problem.ni+50
            problem.yr{i} = [4;-4];
        end
        
    case 3
        problem.ni = 100;
        for i = 1:problem.ni+25
            problem.yr{i} = [0;0];
        end
        for i = problem.ni+26:problem.ni+50
            problem.yr{i} = [4;-4];
        end

    case 4
        
        problem.umin = -0.4;
        problem.umax =  0.4;
        problem.ni = 10;
        %problem.uIdx = [1 2];
        
        for i = 1:problem.ni+25
            problem.yr{i} = [0;0];
        end
        for i = problem.ni+40:problem.ni+100
            problem.yr{i} = [4;-4];
        end

    otherwise
        
        error( 'Invalid variant number!' );
        
end

end

