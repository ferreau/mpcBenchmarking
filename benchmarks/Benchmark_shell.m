function [ problem ] = Benchmark_shell( variant )
%Benchmark_shell does something.
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
problem = problem.loadFromMatFile( 'shell',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.shell);
problem.info.name         = 'shell';
problem.info.description  = 'Linearized model of the Shell heavy oil fractionator plant. The system is highly coupled and has time delays.';
problem.info.reference    = 'D.M. Prett, M. Morari: The Shell Process Control Workshop, Butterworths, Boston, 1987.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.yes;


%% define MPC benchmark data

% example of a simple LTI system
s = tf('s');
g11 = 4.05*exp(-27*s)/(50*s + 1);
g12 = 1.77*exp(-28*s)/(60*s + 1);
g13 = 5.88*exp(-27*s)/(50*s + 1);
g21 = 5.39*exp(-18*s)/(50*s + 1);
g22 = 5.72*exp(-14*s)/(60*s + 1);
g23 = 6.90*exp(-15*s)/(40*s + 1);
g31 = 4.38*exp(-20*s)/(33*s + 1);
g32 = 4.42*exp(-22*s)/(44*s + 1);
g33 = 7.20/(19*s + 1);

Gc = [g11 g12 g13; g21 g22 g23; g31 g32 g33];

sysC = ss(Gc);
sys = c2d(sysC,3);
[A,B,C,D] = ssdata(sys);


problem.A = A;        % compulsory
problem.B = B;        % compulsory
problem.C = C;            % optional (default, identity matrix)
problem.D = D;        % optional (default, zero matrix)

problem.umax =  [0.5; 0.5; 0.5];             % all constraint fields are optional
problem.umin = -[0.5; 0.5; 0.5];             % (default values -inf/inf)
%problem.dmax =  ;             % all constraint fields are optional
%problem.dmin = -[0.05; 0.05; 0.05];             % (default values -inf/inf)
problem.ymax = [0.5; 0.5; 0.5];
problem.ymin = -[0.5; 0.5; 0.5];       % same for dmin,dmax,M,N,dNmin,dNmax,T

problem.Q  = 100*eye(3);           % optional (default, identity matrix)
problem.R  = 1*eye(3);              % optional (default, zero matrix)
%problem.S  = zeros(2,1);       % optional (default, zero matrix)
%problem.gu = 0;                % optional (default, zero vector)
%problem.gy = zeros(2,1);       % optional (default, zero vector)

%{
problem.deltaInput = setupDeltaInputStruct();
problem.deltaInput.active = Boolean.yes;
problem.deltaInput.weight = problem.R;
problem.deltaInput.dumax =  [0.05; 0.05; 0.05];
problem.deltaInput.dumin = -[0.05; 0.05; 0.05];
%}

% [~, problem.P] = dlqr(problem.A, problem.B, problem.Q, ...
%     problem.R, problem.S);     % optional (default, zero matrix)


% problem.yr  = zeros(3,1);      % optional (default, zero vector)
% problem.ur  = 0;               % optional (default, zero vector)
% problem.xNr = zeros(2,1);      % optional (default, zero vector)


problem.x0 = [1 0.2 1 0.2 0.3 0.1 0 0.2 0.22]';     % compulsory      

problem.ni   = 20;                  % compulsory      
problem.uIdx = [];                  % optional (default, []. Control horizon same as prediction horizon)
problem.lookAhead = Boolean.yes;    % optional (default, 0.  No access to future value of reference trajectories)
problem.simModel  = [];             % optional (default, simulation model same as prediction model)

problem.Ts = 60;

%% define control scenario
problem.variants = [1 2];

switch ( variant )
    
    case 1
        
        for i=1:problem.ni+30
            problem.yr{i} = [0;0;0];
        end


	case 2
        problem.ni = 40;
        for i=1:problem.ni+100
            problem.yr{i} = [0;0;0];
        end

        
    otherwise
        error( 'Invalid variant number!' );
end

end
