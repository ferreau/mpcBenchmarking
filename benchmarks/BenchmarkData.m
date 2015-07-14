classdef BenchmarkData
%BenchmarkData class for collecting public user data for setting up MPC benchmark problems.
%
%See also Benchmark.

% This file is part of the CAESAR MPC Suite developed at 
% ABB Corporate Research (CHCRC.C1).
% It is distributed under the terms of the Eclipse Public License v1.0,
% see the file LICENSE in the root directory.
%
% Authors:         Joachim Ferreau, Helfried Peyrl, 
%                  Dimitris Kouzoupis, Andrea Zanelli
% Last modified:   14/7/2015

    
    properties  
        
        info        % benchmark information (see setupBenchmarkInfoStruct.m)
        variants    % all available benchmark variants
        curVariant  % benchmark variant (used to properly name the problem folders generated from fiordos and forces)

        
        % Linear dynamics
        
        A           % x[k+1] = A[k]x[k] + B[k]u[k] + f[k] 
        B           % y[k]   = C[k]x[k] + D[k]u[k] + e[k]
        f
        C
        D
        e
        
        % Objective function
                    % min SUM_{k=0}_{N-1} [ 
        Q           %   (y[k]-y_r[k])'Q[k](y[k]-y_r[k]) +
        R           % + (u[k]-u_r[k])'R[k](u[k]-u_r[k]) +
        S           % + (y[k]-y_r[k])'S[k](u[k]-u_r[k]) +
        gy          % + (g_y[k]')(y[k]-y_r[k]) +
        gu          % + (g_u[k]')(u[k]-u_r[k]) ] +                  
        P           % + (x[N]-x_r[N])'P(x[N]-x_r[N])               
        
        % Reference values
        
        yr          % reference output trajectory
        ur          % reference input trajectory
        xNr         % reference terminal state
        
        % State and input constraints
        
        ymin        % ymin[k] <= y[k] <= ymax[k]
        ymax
        umin        % umin[k] <= u[k] <= umax[k]
        umax
        dmin        % dmin[k] <= M[k]y[k] + N[k]u[k] <= dmax[k]
        dmax
        M
        N
        dNmin       % dNmin   <= Tx[N] <= dNmax
        dNmax
        T
        
        % special formulations
        
        soft                % TODO: remove this copy of old struct, temporarily kept for backwards compatibility with K project
        softConstraints     % struct containing information on use of soft constraints (see setupSoftConstraintsStruct.m)
        deltaInput          % struct containing information on use Delta-U formulations (see setupDeltaInputStruct.m)
                    
        
        % Other properties
        ni          % horizon length        
        uIdx        % indices where control actions are allowed to move (if empty, 1:ni is assumed)
        lookAhead   % access to future trajectory values
        
        x0          % initial condition
        
        Ts           % sampling time
        timeInstants % Isolate time instants to be solved from the series of QPs
        
        simModel    % name of Matlab function to simulate model
                    % (if empty, linear simulation model as used for
                    % controller is employed, i.e. no plant-model mismatch)
               
    end
    
 
       
    methods ( Access = public ) 

        % constructor
        function obj = BenchmarkData( curVar )
                       
            if ~isnumeric(curVar)
                curVar = str2double(curVar);
            end
            curVar = round(abs(curVar));

            obj.curVariant = curVar;

        end

    end
    
end
