function [ problem ] = Benchmark_spacecraft( variant )
%Benchmark_spacecraft does something.
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
problem = problem.loadFromMatFile( 'spacecraft',variant );

problem.info              = setupBenchmarkInfoStruct( );
problem.info.ID           = uint32(Benchmarks.spacecraft);
problem.info.name         = 'spacecraft';
problem.info.description  = 'Spacecraft attitude control problem.';
problem.info.reference    = 'O. Hegrenaes, J.T. Gravdahl, P. Tondel: Spacecraft attitude control using explicit model predictive control, Automatica, vol. 41, pp. 2107–2114, 2005.';
problem.info.origin       = Origin.academicExample;
problem.info.conditioning = Conditioning.undefined;
problem.info.feasibility  = Feasibility.undefined;
problem.info.isOpenLoopStable = Boolean.no;


%% define MPC benchmark data
% Parameters:
problem.Ts = 0.1;
w0 = 1;
i11 = 4.250;
i22 = 4.337;
i33 = 3.664;
Iw = 4.0e-5;
k = i22 - Iw;
kx = (i22 - i33)/i11;
ky = (i11 - i33)/i22;
kz = (i22 - i11)/i33;

% example of a simple LTI system
Ac = [  0               0               (1-kx)*w0           0              -8*kx*w0^2           0                   0;          ...
        0               0               0                   0               0                  -6*ky*i22*w0^2/k     0;          ...
        (kz -1)*w0      0               0                   0               0                   0                  -2*kz*w0^2;  ...
        0               0               0                   0               0                  -6*ky*i22*w0^2/k     0;          ...
        0.5             0               0                   0               0                   0                   0;          ...
        0               0.5             0                   0               0                   0                   0;          ...
        0               0               0.5                 0               0                   0                   0];
    
 
    
Bc = [  1/i11           0               0                   0;          ...
        0               1/k             0                   -1/k;       ... 
        0               0               1/i33               0;          ...
        0               -1/k            0                   i22/k*Iw;   ...
        0               0               0                   0;          ...
        0               0               0                   0;          ...
        0               0               0                   0];

    
sysd = c2d(ss(Ac,Bc,eye(7),zeros(7,4)),problem.Ts);
[problem.A, problem.B, problem.C,problem.D] = ssdata(sysd);

problem.umax =  [   0.0484;    0.0484;     0.0398;      0.0020   ];             
problem.umin = -[   0.0484;    0.0484;     0.0398;      0.0020   ];
problem.ymax =  [   1;          1;         1;           800;        1;      1;      1   ];
problem.ymin = -[   1;          1;         1;           800;        1;      1;      1   ];

problem.Q  = diag([500,500,500,1.0e-7,1,1,1]);
problem.R  = diag([200,200,200,1]);              


problem.x0 = 0.1*ones(7,1);     % compulsory      

problem.ni   = 10;                  % compulsory      
problem.uIdx = [];                  % optional (default, []. Control horizon same as prediction horizon)
problem.lookAhead = Boolean.yes;    % optional (default, 0.  No access to future value of reference trajectories)
problem.simModel  = [];             % optional (default, simulation model same as prediction model)


%% define control scenario
problem.variants = [1 2];

switch ( variant )
    
    case 1
        problem.uIdx = [1 2];
        for i = 1:problem.ni+500
            problem.yr{i} = [0;0;0;0;0;0;0];
        end
        
    case 2
        for i = 1:problem.ni+500
            problem.yr{i} = [0;0;0;0;0;0;0];
        end
      
    otherwise
        error( 'Invalid variant number!' );
end

end
