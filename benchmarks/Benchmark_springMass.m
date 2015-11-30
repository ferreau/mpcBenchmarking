function [ problem ] = Benchmark_springMass( variant )
%Benchmark_springMass does something.
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
problem = problem.loadFromMatFile( 'springMass',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.springMass);
problem.info.name         = 'springMass';
problem.info.description  = 'Spring-mass system with input and state constraints.';
problem.info.reference    = 'S. Longo, E.C. Kerrigan, G.A. Constantinides: A predictive Control Solver for Low-Precision Data Represenation, Proceedings of the 2013 European Control Conference (ECC), pp. 3590-3595, 2013, Zurich, Switzerland.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.no;


%% define MPC problem data

% Continuous time model
% x_dot(t) = Ac*x(t) + Bc*u(t)

Ac = [ 0    0    0   -2    1    0  ;
       0    0    0    1   -2    1  ; 
       0    0    0    0    1   -2  ;
       1    0    0    0    0    0  ;
       0    1    0    0    0    0  ;
       0    0    1    0    0    0 ];

Bc = [ 1    0  ;
       0    0  ;
       0    0  ; 
       0    0  ;
       0    0  ;
       0    1 ];

Cc  = eye(6);

Dc  = zeros(6,2);

problem.Ts = 0.01; % Sampling time [s]  

sysd = c2d(ss(Ac,Bc,Cc,Dc),problem.Ts);
[problem.A, problem.B, problem.C,problem.D] = ssdata(sysd);

problem.umin = [-0.5 -0.5]';                     % infinity norm of u[k] <= 0.5
problem.umax = [ 0.5  0.5]';
problem.ymin = [-3.5 -3.5 -3.5 -3.5 -3.5 -3.5]'; % infinity norm of x[k] <= 3.5
problem.ymax = [ 3.5  3.5  3.5  3.5  3.5  3.5]'; 

% In literature, Q,R,S,P are derived from the discretization of the
% continuous-time LQ with parameters:
Qc = eye(6);
Rc = eye(2);
Sc = zeros(6,2);
[~,Pc] = lqr(Ac,Bc,Qc,Rc,Sc);  

% for now:
problem.Q  = eye(6);
problem.R  = eye(2);
[~,problem.P] = dlqr(problem.A,problem.B,problem.Q,problem.R);


%% define control scenario
problem.variants = [1 2 3 4];

switch variant
    
    case 1
        % close to literature variant
        % adjusted x0 to yield feasible QPs
        problem.ni = 200;
        problem.x0 = 0.88*[0 0 3.5 3.5 0 0]';
        for i = 1:problem.ni+1500
            problem.yr{i} = [0;0;0;0;0;0];
        end
        
    case 2
        % 'easier variant' in closed loop
        problem.ni = 20;
        problem.x0 = 0.5*[0 0 3.5 3.5 0 0]';
        for i = 1:problem.ni+500
            problem.yr{i} = [0;0;0;0;0;0];
        end
        
    case 3
        % 'easier variant' in closed loop and diagonal terminal weight
        problem.ni = 20;
        problem.x0 = 0.5*[0 0 3.5 3.5 0 0]';
        for i = 1:problem.ni+500
            problem.yr{i} = [0;0;0;0;0;0];
        end
        problem.P = diag(diag(problem.P));

    case 4
        % 'easier variant' in closed loop with somewhat longer horizon
        problem.ni = 40;
        problem.x0 = 0.5*[0 0 3.5 3.5 0 0]';
        for i = 1:problem.ni+500
            problem.yr{i} = [0;0;0;0;0;0];
        end
               
    otherwise
        error( 'Invalid variant number!' );
end

end
