classdef Benchmark < BenchmarkData
%Benchmark class for setting up MPC benchmark problems.
%          Inherits data members from class BenchmarkData.
%
%See also BenchmarkData.

% This file is part of the CAESAR MPC Suite developed at 
% ABB Corporate Research (CHCRC.C1).
% It is distributed under the terms of the Eclipse Public License v1.0,
% see the file LICENSE in the root directory.
%
% Authors:         Joachim Ferreau, Helfried Peyrl, 
%                  Dimitris Kouzoupis, Andrea Zanelli
% Last modified:   14/7/2015

    
    properties  

        % Benchmark dimensions
        nx          % number of states
        nu          % number of inputs
        ny          % number of outputs
        
        scenario    % simulation scenario (value assigned in divide method) 
                    % 0: Single problem instance
                    % 1: RHC
                    % 2: Multiple initial conditions
                    % 3: Data stored in blocks??
                    
        uPrevious   % previous control input, for deltaInput formulation and blocks of data (e.g. in K project)
        
    end
    
    properties ( SetAccess = protected ) % read-only properties
       
        isinit % is the benchmark initialized

    end
    
    properties ( Hidden, SetAccess = protected ) % read-only, hidden properties
        
        moveBlocking     % internal struct containing information on move blocking (see setupMoveBlockingStruct.m)
        
        
        % Benchmark varying data flags
        
        flagVaryingA
        flagVaryingB
        flagVaryingf
        flagVaryingC
        flagVaryingD
        flagVaryinge
        
        flagVaryingQuadraticWeights          % Q,R,S,P
        flagVaryingLinearWeights             % gy,gu
        flagVaryingPolytopicConstraints      % M,N   
        
    end
 
       
    methods ( Access = public ) 

        % constructor
        function obj = Benchmark( arg )
            
            if isa( arg,'BenchmarkData' )
                curVar = arg.curVariant;
            else
                curVar = arg;
            end
            
            obj = obj@BenchmarkData( curVar );
            obj.isinit = 0;
            
            % copy all members to 
            if isa( arg,'BenchmarkData' )
                
                allFields = fields(arg);
                
                for ii=1:length(allFields)
                    curField = allFields{ii};
                    eval( ['obj.',curField,' = arg.',curField,';'] );
                end
                
            end

        end

        
        % initialize benchmark
        function obj = initialize( obj )
                   
            % Check horizon length
            if isempty(obj.ni) || ( obj.ni == 0 ) || ( obj.ni < 0 ) || ~isreal(obj.ni)
                error( 'Invalid horizon length!' );
            end
            
            if isempty(obj.Ts)
                obj.Ts = 1;
            end
            
            % Default look-ahead option
            if isempty(obj.lookAhead)
                obj.lookAhead = Boolean.no;
            end
            
            % Convert matrix (of merged vectors) to cells
            if ~iscell(obj.x0) && size(obj.x0,2) > 1
                obj.x0 = mat2cell(obj.x0,size(obj.x0,1),ones(1,size(obj.x0,2)));
            end

            % Initialize dynamics
            obj = initDynamics(obj);

            % fill LTV dynamics
            if ~obj.isLTI()
                obj = fillDynamics(obj);
            end

            % Initialize cost function
            obj = initCost(obj);
            
            % fill TV cost function quantities
            if any([iscell(obj.Q)  iscell(obj.R) iscell(obj.S) ...
                    iscell(obj.gy) iscell(obj.gu)]); % TODO: can make it a dependent property like islti in the future
                
                obj = fillCost(obj);  
            end
                        
            % Initialize constraints
            obj = initConstraints(obj);
            
            % fill TV constraints
            if any([iscell(obj.ymin) iscell(obj.ymax) iscell(obj.umin) ...
                    iscell(obj.umax) iscell(obj.dmin) iscell(obj.dmax) ...
                    iscell(obj.M)    iscell(obj.N)])
               
                obj = fillConstraints(obj);
            end
               
            % Initialize trajectories
            obj = initTrajectories(obj);
            
            % fill TV trajectories (always convert to cells)
            obj = fillTrajectories(obj);
            

            % Create a cell with different representations of Nc
            if obj.usesMoveBlocking()

                obj.moveBlocking = setupMoveBlockingStruct();
                
                % first representation: indices of time steps where control action is allows to move (starting from 1)
                obj.moveBlocking.uIdx = obj.uIdx;
                if ( obj.moveBlocking.uIdx(1) ~= 1 )
                    error( 'First time step (1) must belong to control horizon!' );
                end
                % do not allow control after end of horizon
                obj.moveBlocking.uIdx(obj.moveBlocking.uIdx > obj.ni-1) = [];
                
                % second representation: number of time steps with the same control (add up to ni)
                tmp = obj.uIdx;
                tmp(end+1) = obj.ni + 1;
                tmp = circshift(tmp',-1)' - tmp;
                tmp(end) = [];
                obj.moveBlocking.uCount = tmp;

                % third representation: vector indicating the time steps where control action is allows to move
                obj.moveBlocking.uIndVec = zeros(1,obj.ni);
                obj.moveBlocking.uIndVec( obj.moveBlocking.uIdx ) = 1;

                % fourth representation: matrix indicating the time steps where control action is allows to move
                patt = zeros(length(tmp),obj.ni);

                j = 1;
                for i = 1:length(tmp)
                    patt(i,j:j+tmp(i)-1) = 1;
                    j = j + tmp(i);
                end
                obj.moveBlocking.uIndMat = patt;                   
                
            end

            % Initialization for soft constraints
            if ~obj.hasSoftConstraints()
                
                % no soft constraints 
                obj.softConstraints = setupSoftConstraintsStruct();
                
            else
                
                % check index value
                if isempty(obj.softConstraints.index)
                    obj.softConstraints.index = zeros(obj.ny,1);
                end
                if length(obj.softConstraints.index) ~= obj.ny
                    error('soft constraints index has different dimension than the ouput vector');
                end
                obj.softConstraints.index = logical(obj.softConstraints.index);
                
                % check horizon flag
                if isempty(obj.softConstraints.alongHorizon)
                    obj.softConstraints.alongHorizon = Boolean.no; % one slack for whole interval
                else
                    obj.softConstraints.alongHorizon = Boolean(logical(obj.softConstraints.alongHorizon));
                end
                
                % check linear weights
                if iscell(obj.softConstraints.linWeight)
                    error('Varying slack weights NIY') % TODO
                end
                if isempty(obj.softConstraints.linWeight) && any(obj.softConstraints.index)
                    % set to zero if uninitialized
                    obj.softConstraints.linWeight = zeros(obj.ny,1);
                    % hard-constrained outputs are assigned nan weights
                    obj.softConstraints.linWeight(~obj.softConstraints.index) = NaN;
                end
                if ~isvector(obj.softConstraints.linWeight)
                    error('Linear weights on slack variables must be given in vector form')
                end
                if length(obj.softConstraints.linWeight) < obj.ny
                    % maybe only weights for soft inputs are defined by user
                    if length(obj.softConstraints.linWeight) < sum(obj.softConstraints.index)
                        error('Not enough linear weights provided')
                    else
                        linWeight = nan(obj.ny,1);
                        linWeight(obj.softConstraints.index) = obj.softConstraints.linWeight;
                        obj.softConstraints.linWeight = linWeight;
                    end
                elseif length(obj.softConstraints.linWeight) == obj.ny
                    % set to nan all weights for non-soft-constrained outputs
                    obj.softConstraints.linWeight(~obj.softConstraints.index) = NaN;
                else
                    error('Wrong number of linear weights for slack variables');  
                end
                
                if ~isempty(obj.softConstraints.linWeight)
                    % ensure weights are in column vector
                    obj.softConstraints.linWeight = obj.softConstraints.linWeight(:);
                end
                
                % check quadratic weights
                if iscell(obj.softConstraints.quadWeight)
                    error('Varying slack weights NIY') % TODO
                end
                if isempty(obj.softConstraints.quadWeight) && any(obj.softConstraints.index)
                    obj.softConstraints.quadWeight = zeros(obj.ny,1);
                    obj.softConstraints.quadWeight(~obj.softConstraints.index) = nan;
                end
                if ~isvector(obj.softConstraints.quadWeight) 
                    % convert given matrix to vector, if diagonal
                    if ~isequal(diag(diag(obj.softConstraints.quadWeight)),obj.softConstraints.quadWeight)
                        error('Quadratic weight on slack variables must be diagonal');
                    end
                    obj.softConstraints.quadWeight = diag(obj.softConstraints.quadWeight);
                end
                if length(obj.softConstraints.quadWeight) < obj.ny
                    % maybe only weights for soft inputs are defined by user
                    if length(obj.softConstraints.quadWeight) < sum(obj.softConstraints.index)
                        error('Not enough quadratic weights provided')
                    else
                        quadWeight = nan(obj.ny,1);
                        quadWeight(obj.softConstraints.index) = obj.softConstraints.quadWeight;
                        obj.softConstraints.quadWeight = quadWeight;
                    end
                elseif length(obj.softConstraints.quadWeight) == obj.ny
                    % set to nan all weights for non-soft-constrained outputs
                    obj.softConstraints.quadWeight(~obj.softConstraints.index) = nan;
                else
                    error('Wrong number of quadratic weights for slack variables');  
                end
                if ~isempty(obj.softConstraints.quadWeight)
                    % ensure weights are in column vector
                    obj.softConstraints.quadWeight = obj.softConstraints.quadWeight(:);
                end
            end
            
            if ( ( ~isempty(obj.simModel) ) && ( exist(obj.simModel,'file') == 0 ) )
                error( 'Cannot find model function for simulation! Please correct value of ''simModel''.' );
            end
            
            obj.isinit = 1;
            
        end
        
        
        % divide benchmark in sub-problems
        function subobj = divide( obj )
            %{
            DIVIDEPROBLEMSTRUCT: Divide initial problem structure in 
            subproblems that can be converted into QPs. Addressed cases:
        
            a. Multiple initial conditions. Seperate into structures that 
               contain the same problem but different initial condition           
        
            b. Reference trajectories or system dynamics longer than 
               horizon. Seperate into receding horizon problems until end 
               of given data
        
            c. Data already divided into subproblems. Do nothing
        
            d. None of the above. There is only one problem and therefore 
               no need to divide into multiple structs
            %}
            
            % obj.scenario = 0: single QP
            % obj.scenario = 1: RHC
            % obj.scenario = 2: multiple x0
            % obj.scenario = 3: blocks of data
            
            % Data already divided in subproblems. Exiting function
            if length(obj) > 1
                subobj = obj;
                subobj{1}.scenario = 3; % TODO: write a get to only return scenario of first problem
                return;
            end
            
            % Multiple initial conditions
            if iscell(obj.x0)
                
                np = length(obj.x0); % number of subproblems
                
                % keep only the N first data of the problem
                obj = trimBenchmark(obj, obj.ni);
                
                subobj = cell(np,1);
                
                for i = 1:np
                    
                    x0loc = obj.x0{i};

                    subobj{i}    = Benchmark();
                    subobj{i}    = obj;
                    subobj{i}.x0 = [];
                    subobj{i}.x0 = x0loc;
                    
                end
                
                subobj{1}.scenario = 2;
                
            % Given output (and optionally input) reference trajectory
            % longer than horizon
            elseif iscell(obj.yr) && length(obj.yr) > obj.ni
                
                np = length(obj.yr) - obj.ni + 1;
                subobj = cell(np,1);
                
                % in case of an LTV system, dynamics must be at least as long as the provided reference signals
                if ~obj.isLTI()
                    if  length(obj.yr) <= length(obj.A)
                        obj = trimProblem(obj,length(obj.yr));
                    else
                        error('For LTV systems, reference signal can not be longer than the provided dynamics');
                    end
                end
                
                
                for i = 1:np
                    
                    % shift reference signals
                    yref = obj.yr(i:i+obj.ni-1);
                    uref = obj.ur(i:i+obj.ni-1);
                    
                    % shift system matrices for LTV systems
                    if ~obj.isLTI()
                        Aloc = obj.A(i:i+obj.ni-1);
                        Bloc = obj.B(i:i+obj.ni-1);
                        floc = obj.f(i:i+obj.ni-1);
                        Cloc = obj.C(i:i+obj.ni-1);
                        Dloc = obj.D(i:i+obj.ni-1);
                        eloc = obj.e(i:i+obj.ni-1);
                    end
                    
                    subobj{i}    = obj;
                    subobj{i}.yr = [];
                    subobj{i}.ur = [];
                    subobj{i}.yr = yref;
                    subobj{i}.ur = uref;
                    
                    if ~obj.isLTI()
                        subobj{i}.A(:) = [];
                        subobj{i}.B(:) = [];
                        subobj{i}.f(:) = [];
                        subobj{i}.C(:) = [];
                        subobj{i}.D(:) = [];
                        subobj{i}.A    = Aloc;
                        subobj{i}.B    = Bloc;
                        subobj{i}.f    = floc;
                        subobj{i}.C    = Cloc;
                        subobj{i}.D    = Dloc;
                        subobj{i}.e    = eloc;
                    end
                    
                    % TODO: in the future, might also shift constraints and cost function elements  
                    
                    if i>1
                        % delete initial condition since RH is parametric wrt x0
                        subobj{i}.x0 = [];
                    end
                    
                end
                
                subobj{1}.scenario  = 1;
                
            % Provided system dynamics longer than horizon
            elseif iscell(obj.A) && length(obj.A) > obj.ni
                
                % define number of subproblems and struct array
                np = length(obj.A) - obj.ni + 1;
                subobj = cell(np,1);
                
                % extend uref and yref
                if iscell(obj.yr)
                    if length(obj.ur) < length(obj.A)
                        for i = length(obj.ur) + 1 : length(obj.A)
                            obj.ur{i} = obj.ur{i-1};
                        end
                    end
                    if length(obj.yr) < length(obj.A)
                        for i = length(obj.yr) + 1 : length(obj.A)
                            obj.yr{i} = obj.yr{i-1};
                        end
                    end
                end
                
                % store the RH subobjs
                for i = 1:np
                    
                    % shift system matrices
                    Aloc = obj.A(i:i+obj.ni-1);
                    Bloc = obj.B(i:i+obj.ni-1);
                    floc = obj.f(i:i+obj.ni-1);
                    Cloc = obj.C(i:i+obj.ni-1);
                    Dloc = obj.D(i:i+obj.ni-1);
                    
                    % shift reference signals
                    yref = obj.yr(i:i+obj.ni-1);
                    uref = obj.ur(i:i+obj.ni-1);
                    
                    subobj{i}    = obj;
                    subobj{i}.yr = [];
                    subobj{i}.ur = [];
                    subobj{i}.yr = yref;
                    subobj{i}.ur = uref;
                    subobj{i}.A  = [];
                    subobj{i}.B  = [];
                    subobj{i}.f  = [];
                    subobj{i}.C  = [];
                    subobj{i}.D  = [];
                    subobj{i}.A  = Aloc;
                    subobj{i}.B  = Bloc;
                    subobj{i}.f  = floc;
                    subobj{i}.C  = Cloc;
                    subobj{i}.D  = Dloc;
                    
                    if i>1
                        % delete initial condition since RH is parametric wrt x0
                        subobj{i}.x0 = [];
                    end
                    
                end
                
                subobj{1}.scenario  = 1;
            
            % Only one single QP
            else
                subobj = {obj};
                subobj{1}.scenario = 0;                
            end
            
        end
 
        
        function [ string ] = getName( obj )
            
            if ( isempty(obj.info) )
                string = 'untitled';
            else
                string = obj.info.name;
            end
            
        end
        
        
        function [ string ] = getLabel( obj,variant )
            
            if ( nargin < 2 )
                string = [obj.getName(),'_',num2str(obj.curVariant)];
            else
                string = [obj.getName(),'_',num2str(variant)];
            end
            
        end
        
        
        function [ nVariants ] = getNumVariants( obj )
            
            nVariants = length(obj.variants);
            
        end
        
        
        function [ answer ] = hasPredefinedScenario( obj )
            
            if ( ~isempty(obj.scenario) ) && ( obj.scenario == 3 )
                answer = Boolean.yes;
            else
                answer = Boolean.no;
            end
            
        end

        
        % overloaded isempty function to accept cell and non-cell inputs
        function value = isempty( obj,prop )
        
            
            if nargin == 1
                value = builtin('isempty',obj);
                return
            end
            
            answer = eval(['iscell(obj.' prop ');']);
            
            if answer == 0
                
                value = eval(['builtin(''isempty'', obj.' prop ');']);
                
            else
                
                value = eval(['builtin(''isempty'', obj.' prop '{1});']);
                
            end
            
        end

        
        function [ answer ] = hasDenseHessian( obj )
            
            if ( ( obj.hasDiagonalHessian() == 0 ) && ( obj.hasZeroHessian() == 0 ) )
                answer = 1;
            else
                answer = 0;
            end
            
        end

        
        function [ answer ] = hasDiagonalHessianWithP( obj )
            
            answer = 1;
            
            % analyse Q
            if ( ~iscell(obj.Q) )
                if ( isdiag(obj.Q) == 0 ) 
                    answer = 0;
                    return;
                end
            else
                for ii=1:length(obj.Q)
                    if ( isdiag(obj.Q{ii}) == 0 )
                        answer = 0;
                        return;
                    end
                end
            end
            
            % analyse R
            if ( ~iscell(obj.R) )
                if ( isdiag(obj.R) == 0 ) 
                    answer = 0;
                    return;
                end
            else
                for ii=1:length(obj.R)
                    if ( isdiag(obj.R{ii}) == 0 )
                        answer = 0;
                        return;
                    end
                end
            end
                       
            % analyse S
            if ( ~iscell(obj.S) )
                if ( iszero(obj.S) == 0 ) 
                    answer = 0;
                end
            else
                for ii=1:length(obj.S)
                    if ( iszero(obj.S{ii}) == 0 )
                        answer = 0;
                        break;
                    end
                end
            end
            
        end


        function [ answer ] = hasDiagonalHessian( obj )
            
            answer = obj.hasDiagonalHessianWithP();
            if ( answer == 0 )
                return;
            end
                        
            % analyse P
            if ( ~iscell(obj.P) )
                if ( isdiag(obj.P) == 0 ) 
                    answer = 0;
                    return;
                end
            else
                if ( isdiag(obj.P{1}) == 0 )
                    answer = 0;
                    return;
                end
            end
            
        end


        function [ answer ] = hasZeroHessian( obj )
            
            answer = 1;
            
            % analyse Q
            if ( ~iscell(obj.Q) )
                if ( iszero(obj.Q) == 0 ) 
                    answer = 0;
                    return;
                end
            else
                for ii=1:length(obj.Q)
                    if ( iszero(obj.Q{ii}) == 0 )
                        answer = 0;
                        return;
                    end
                end
            end
            
            % analyse R
            if ( ~iscell(obj.R) )
                if ( iszero(obj.R) == 0 ) 
                    answer = 0;
                    return;
                end
            else
                for ii=1:length(obj.R)
                    if ( iszero(obj.R{ii}) == 0 )
                        answer = 0;
                        return;
                    end
                end
            end
            
            % analyse P
            if ( ~iscell(obj.P) )
                if ( iszero(obj.P) == 0 ) 
                    answer = 0;
                    return;
                end
            else
                if ( iszero(obj.P{1}) == 0 )
                    answer = 0;
                    return;
                end
            end
            
            % analyse S
            if ( ~iscell(obj.S) )
                if ( iszero(obj.S) == 0 ) 
                    answer = 0;
                end
            else
                for ii=1:length(obj.S)
                    if ( iszero(obj.S{ii}) == 0 )
                        answer = 0;
                        break;
                    end
                end
            end
            
        end
        
        
        function [ answer ] = isPosDefDense( obj )
            
            answer = 1;
            
            % analyse R
            if ( ~iscell(obj.R) )
                if ( ~isempty(obj.R) && ( isposdef(obj.R) == 0 ) )
                    answer = 0;
                    return;
                end
            else
                for ii=1:length(obj.R)
                    if ( ~isempty(obj.R) && ( isposdef(obj.R{ii}) == 0 ) )
                        answer = 0;
                        return;
                    end
                end
            end
            
        end

        
        function [ answer ] = isPosDef( obj )
            
            answer = obj.isPosDefDense();
            if ( answer == 0 )
                return;
            end
            
            % analyse Q
            if ( ~iscell(obj.Q) )
                if ( isposdef(obj.Q) == 0 ) 
                    answer = 0;
                    return;
                end
            else
                for ii=1:length(obj.Q)
                    if ( isposdef(obj.Q{ii}) == 0 )
                        answer = 0;
                        return;
                    end
                end
            end
            
            % analyse P
            if ( ~iscell(obj.P) )
                if ( isposdef(obj.P) == 0 ) 
                    answer = 0;
                    return;
                end
            else
                if ( isposdef(obj.P{1}) == 0 )
                    answer = 0;
                    return;
                end
            end
            
        end

        
        function [ answer ] = isPosSemDef( obj )
            
            if ( obj.isPosDef() == 0 )
                answer = 1;
            else
                answer = 0;
            end
            
        end

        
        % is the system LTI
        function [ answer ] = isLTI( obj )

            
            if ~iscell(obj.A) && ~iscell(obj.B) && ~iscell(obj.f) && ...
                    ~iscell(obj.C) && ~iscell(obj.D) && ~iscell(obj.e)
                answer = 1;
            else
                answer = 0;
            end
            
        end

        
        function [ answer ] = isLTV( obj )
        % is the system LTV
            
            if ~isLTI(obj)
                answer = 1;
            else
                answer = 0;
            end
            
        end
        
        
        function [ answer ] = hasNonlinearDynamics( obj ) %#ok<MANU>
        % has the system nonlinear dynamics
            
            answer = 0;
            
        end

        
        % is full state information available (y = x)
        function [ answer ] = hasFullState( obj )
        
            if ~obj.isinit
                s = warning();
                warning('off','all')
                obj = obj.initialize();
                warning(s)
            end
            
            answer = 1;
            
            if ~obj.isLTI()
                dim  = length(obj.C);
            else
                dim  = 1;
            end
            
            for i = 1:dim
                
                if ~obj.isLTI()
                    Cloc = obj.C{i};
                    Dloc = obj.D{i};
                else
                    Cloc = obj.C;
                    Dloc = obj.D;
                end
                
                if any(size(Cloc) ~= [obj.nx obj.nx])
                    answer = 0;
                end
                
                if all(size(Cloc) == [obj.nx obj.nx]) && ~iszero(Cloc - eye(obj.nx))
                    answer = 0;
                end
                
                if ~iszero(Dloc)
                    answer = 0;
                end
                
            end
            
            
        end
        
        
        function [ answer ] = hasInputConstraints( obj )
        % does the benchmark have input box constraints
        
            if ~obj.isinit
                s = warning();
                warning('off','all')
                obj = obj.initialize();
                warning(s)
            end
            
            answer = 0;
            if ~iscell(obj.umin)
                if any(~isinf([obj.umin; obj.umax]))
                    answer = 1;
                end
            else
                for i = 1:length(obj.umin)
                    if any(~isinf([obj.umin{i}; obj.umax{i}]))
                        answer = 1;
                    end
                end
            end
            
        end
        

        function [ answer ] = hasOutputConstraints( obj )
        % does the benchmark have output box constraints
            
            if ~obj.isinit
                s = warning();
                warning('off','all')
                obj = obj.initialize();
                warning(s)
            end
            
            answer = 0;
            if ~iscell(obj.ymin)
                if any(~isinf([obj.ymin; obj.ymax]))
                    answer = 1;
                end
            else
                for i = 1:length(obj.ymin)
                    if any(~isinf([obj.ymin{i}; obj.ymax{i}]))
                        answer = 1;
                    end
                end
            end
            
        end
        
        
        function [ answer ] = hasPolytopicConstraints( obj )
        % does the benchmark have polytopic constraints
        
            if ~obj.isinit
                s = warning();
                warning('off','all')
                obj = obj.initialize();
                warning(s)
            end
            
            answer = 0;
            if ~iscell(obj.dmin)
                if any(~isinf([obj.dmin; obj.dmax]))
                    answer = 1;
                end
            else
                for i = 1:length(obj.dmin)
                    if any(~isinf([obj.dmin{i}; obj.dmax{i}]))
                        answer = 1;
                    end
                end
            end
            % flag for identity or zero T
            isTidentity = ~isempty(obj.T) && size(obj.T,1) == size(obj.T,2) && (sum(sum(abs(obj.T-eye(size(obj.T))))) == 0 || sum(sum(abs(obj.T))) == 0 );
            if any(~isinf([obj.dNmin; obj.dNmax])) && ~isTidentity
                answer = 1;
            end
            
        end
        
        
        function [ answer ] = hasSoftConstraints( obj )
        % does the benchmark have output box constraints
            if isempty(obj.softConstraints) || ( sum(obj.softConstraints.index) == 0 )
                answer = 0;
            else
                answer = 1;
            end
        end
        
        
        function [ answer ] = usesDeltaInputFormulation( obj )
        % does the benchmark use a deltaU formulation
            if ( isempty(obj.deltaInput) || ...
                 ( ~isempty(obj.deltaInput) && ( obj.deltaInput.active == Boolean.no ) ) )
                answer = 0;
            else
                answer = 1;
            end
        end
        
        
        function [ answer ] = usesMoveBlocking( obj )
            % does the benchmark use move blocking
            if isempty(obj.uIdx)
                answer = 0;
            else
                if ( length(obj.uIdx) < obj.ni )
                    answer = 1;
                else
                    answer = 0;
                end
            end
        end
        
        
        function [ obj ] = loadFromMatFile( obj,name,variant )

            obj.info.name = name;
            
            if ( nargin < 3 )
                variant = obj.curVariant;
            end
            
            fileName = [ obj.getLabel(variant),'.mat' ];

            if ( exist( fileName,'file' ) > 0 )
                eval( ['tmp = load(''',fileName,''');'] );
                obj = tmp.problem;
                disp( ['Loaded data from file ',fileName] );
%            else
%                error( ['Could not load data from file ',fileName] );
            end

        end

        
        function [ ] = exportToMatFile( obj,variant )

            if ( nargin < 2 )
                variant = obj.curVariant;
            end
            
            if ( obj.isinit )
                error( 'Unable to export benchmark that has been already initialized!' );
            end
            
            if ( any( variant == obj.variants ) == 0 )
                error( ['Unable to export variant no. ',num2str(variant),'!'] );
            end
            
            caesarPath = getCaesarPath( );
            eval( ['problem = Benchmark_',char(obj.getName()),'( variant );'] );
            eval( ['save ',caesarPath,'benchmarks',sep,'matFiles',sep,obj.getLabel(variant),'.mat problem;'] );

        end

        
    end

    
    % read-write varying data flags
    methods ( Access = public )
        
        function [ answer ] = hasVaryingMatrixDynamics( obj )
            
            if isempty(obj.isinit) || obj.isinit == 0 
                s = warning();
                warning('off','all')
                obj = obj.initialize();
                warning(s)
            end
            
            if ( obj.flagVaryingA == 1 ) || ( obj.flagVaryingB == 1 ) || ...
                    ( obj.flagVaryingC == 1 ) || ( obj.flagVaryingD == 1 )
                answer = 1;
            else
                answer = 0;
            end
            
        end
        
        
        function [ answer ] = hasVaryingVectorDynamics( obj )
            
            if isempty(obj.isinit) || obj.isinit == 0
                s = warning();
                warning('off','all')
                obj = obj.initialize();
                warning(s)
            end
            
            if ( obj.flagVaryinge == 1) || ( obj.flagVaryingf == 1 )
                answer = 1;
            else
                answer = 0;
            end
            
        end
        
        
        function [ answer ] = hasVaryingQuadraticWeights( obj )
            
            if isempty(obj.isinit) || obj.isinit == 0
                s = warning();
                warning('off','all')
                obj = obj.initialize();
                warning(s)
            end
            
            if obj.flagVaryingQuadraticWeights
                answer = 1;
            else
                answer = 0;
            end
            
        end
        
        
        function [ answer ] = hasVaryingLinearWeights( obj )
            
            if isempty(obj.isinit) || obj.isinit == 0
                s = warning();
                warning('off','all')
                obj = obj.initialize();
                warning(s)
            end
            
            if obj.flagVaryingLinearWeights
                answer = 1;
            else
                answer = 0;
            end
            
        end
        
        
        function [ answer ] = hasVaryingPolytopicConstraints( obj )
            
            if isempty(obj.isinit) || obj.isinit == 0
                s = warning();
                warning('off','all')
                obj = obj.initialize();
                warning(s)
            end
            
            if obj.flagVaryingPolytopicConstraints
                answer = 1;
            else
                answer = 0;
            end
            
        end
        
        
        function [ answer ] = hasVaryingHessian( obj, formulation )
            
            if isempty(obj.isinit) || obj.isinit == 0
                s = warning();
                warning('off','all')
                obj = obj.initialize();
                warning(s)
            end
            
            answer = 0;
            
            if ( obj.hasVaryingQuadraticWeights() == 1 )
                answer = 1;
                return
            end
            
            switch formulation
                
                case Formulation.condensed
                    
                    if ( obj.hasVaryingMatrixDynamics() == 1 )                     
                        answer = 1;
                    end
                    
                case Formulation.sparseXU
                    
                    if ( obj.flagVaryingC == 1 ) || ( obj.flagVaryingD == 1 )
                        answer = 1;   
                    end
                    
            end

        end
        
        
        % For Kasparov project, we set the flags manually
        function obj = changeFlagVaryingMatrixDynamics(obj, value )
            
            if ( ( value == 0 ) || ( value == 1 ) ) && ( obj.scenario == 3 )
                obj.flagVaryingA = value;
                obj.flagVaryingB = value;
                obj.flagVaryingC = value;
                obj.flagVaryingD = value;
            end
            
        end
        
        
        % For Kasparov project, we set the flags manually
        function obj = changeFlagVaryingVectorDynamics(obj, value )
            
            if ( ( value == 0 ) || ( value == 1 ) ) && ( obj.scenario == 3 )
                obj.flagVaryingf = value;
                obj.flagVaryinge = value;
            end
            
        end
        
        % For Kasparov project, we set the flags manually
        function obj = changeFlagVaryingQuadraticWeights(obj, value )
            
            if ( ( value == 0 ) || ( value == 1 ) ) && ( obj.scenario == 3 )
                obj.flagVaryingQuadraticWeights = value;
            end
            
        end
            
    end
    

    % assistive methods for initialization
    methods ( Access = protected )

        function obj = initDynamics( obj )
        % initialize dynamics

            % Check for empty fields in dynamics
            if isempty(obj,'A') || isempty(obj,'B')
                error('System dynamics missing. Check system matrices A and B')
            end
            
            if ~iscell(obj.B)
                [obj.nx, obj.nu] = size(obj.B);
            else
                [obj.nx, obj.nu] = size(obj.B{1});
            end

            if isempty(obj,'f')
                obj.f = zeros(obj.nx,1);
            end
            
            if isempty(obj,'C')
                obj.C = eye(obj.nx);
            end
            
            if ~iscell(obj.C)
                obj.ny = size(obj.C,1);
            else
                obj.ny = size(obj.C{1},1);
            end
            
            if isempty(obj,'D')
                obj.D = zeros(size(obj.C,1),size(obj.B,2));
            end
            
            if isempty(obj,'e')
                obj.e = zeros(size(obj.C,1),1);
            end
            
            % determine whether the dynamics are varying
            obj.flagVaryingA = 1;
            obj.flagVaryingB = 1;
            obj.flagVaryingf = 1;
            obj.flagVaryingC = 1;
            obj.flagVaryingD = 1;
            obj.flagVaryinge = 1;
            
            if ~iscell(obj.A); obj.flagVaryingA = 0; end
            if ~iscell(obj.B); obj.flagVaryingB = 0; end
            if ~iscell(obj.f); obj.flagVaryingf = 0; end
            if ~iscell(obj.C); obj.flagVaryingC = 0; end
            if ~iscell(obj.D); obj.flagVaryingD = 0; end
            if ~iscell(obj.e); obj.flagVaryinge = 0; end
            
%             if ~iscell(obj.A) && ~iscell(obj.B) && ...
%                     ~iscell(obj.C) && ~iscell(obj.D)
%                 obj.flagVaryingMatrixDynamics = 0;
%             else
%                 obj.flagVaryingMatrixDynamics = 1;
%             end
%             if ~iscell(obj.f) && ~iscell(obj.e)
%                 obj.flagVaryingVectorDynamics = 0;
%             else
%                 obj.flagVaryingVectorDynamics = 1;
%             end

        end
        
        
        function obj = initCost( obj )
        % initialize cost function
            
            if isempty(obj,'Q')
                obj.Q = eye(obj.ny);
                warning('CAESAR:preprocessing','Q matrix not defined, identity matrix used');
            end
            
            if isempty(obj,'R')
                obj.R = zeros(obj.nu);
                warning('CAESAR:preprocessing','R matrix not defined, zero matrix used');
            end
            
            if isempty(obj,'S')
                obj.S = zeros(obj.ny,obj.nu);
            end
            
            if isempty(obj,'P')
                warning('CAESAR:preprocessing','P matrix for terminal state not defined, zero matrix used');
                obj.P = zeros(obj.nx);
            end
            
            if isempty(obj,'gy')
                obj.gy = zeros(obj.ny,1);
            end
            
            if isempty(obj,'gu')
                obj.gu = zeros(obj.nu,1);
            end
            
            % determine whether cost is varying
            if any([iscell(obj.Q)  iscell(obj.R) iscell(obj.S)]);
                obj.flagVaryingQuadraticWeights = 1;
            else
                obj.flagVaryingQuadraticWeights = 0;
            end
            if any([iscell(obj.gy) iscell(obj.gu)]);
                obj.flagVaryingLinearWeights = 1;
            else
                obj.flagVaryingLinearWeights = 0;
            end
            
        end
        
        
        function obj = initConstraints( obj )
        % initialize constraints
            
            if isempty(obj,'M')
                obj.M = zeros(0,obj.ny);
            end
            
            if isempty(obj,'N')
                obj.N = zeros(0,obj.nu);
            end
            
            if isempty(obj,'dmax')
                obj.dmax = zeros(0,1);
            end
            
            if isempty(obj,'dmin')
                obj.dmin = zeros(0,1);
            end
            
            if isempty(obj,'ymax')
                obj.ymax = inf*ones(obj.ny,1);
            end
            
            if isempty(obj,'ymin')
                obj.ymin = -inf*ones(obj.ny,1);
            end
            
            if isempty(obj, 'umax')
                obj.umax = inf*ones(obj.nu,1);
            end
            
            if isempty(obj,'umin')
                obj.umin = -inf*ones(obj.nu,1);
            end
            
            if isempty(obj,'T')
                obj.T = zeros(1,obj.nx);
            end
            
            if isempty(obj,'dNmax')
                obj.dNmax = inf*ones(size(obj.T,1),1);
            end
            
            if isempty(obj,'dNmin')
                obj.dNmin = -inf*ones(size(obj.T,1),1);
            end
            
            % determine whether polyhedral constraints are time-varying
            if any([iscell(obj.M) iscell(obj.N)]);
                obj.flagVaryingPolytopicConstraints = 1;
            else
                obj.flagVaryingPolytopicConstraints = 0;
            end
            
        end
        
        
        function obj = initTrajectories( obj )
        % initialize trajectories
           
            % Convert matrix (of merged vectors) to cells
            if ~iscell(obj.yr) && size(obj.yr,2) > 1
                obj.yr = mat2cell(obj.yr,size(obj.yr,1),ones(1,size(obj.yr,2)));
            end
            
            if ~iscell(obj.ur) && size(obj.ur,2) > 1
                obj.ur = mat2cell(obj.ur,size(obj.ur,1),ones(1,size(obj.ur,2)));
            end
            
            if isempty(obj,'yr')
                obj.yr = zeros(obj.ny,1);
            end
            
            if isempty(obj,'ur')
                obj.ur = zeros(obj.nu,1);
            end
            
            if isempty(obj,'xNr')
                obj.xNr = zeros(obj.nx,1);
            end
            
            if size(obj.xNr,1) ~= obj.nx || size(obj.xNr,2) ~= 1
                error('Dimension mismatch for the given reference value of the terminal state xNr');
            end
            
        end
        
        
        function obj = fillDynamics( obj )
        % fill TV dynamics
            
            % convert all system matrices that are not cells, to cells
            if ~iscell(obj.A)
                obj.A = {obj.A};
            end
            if ~iscell(obj.B)
                obj.B = {obj.B};
            end
            if ~iscell(obj.f)
                obj.f = {obj.f};
            end
            if ~iscell(obj.C)
                obj.C = {obj.C};
            end
            if ~iscell(obj.D)
                obj.D = {obj.D};
            end
            if ~iscell(obj.e)
                obj.e = {obj.e};
            end
            
            nd = max([length(obj.A) length(obj.B) length(obj.f) ...
                length(obj.C) length(obj.D) length(obj.e) obj.ni]);
            
            % extend cells with empty instances
            if length(obj.A) < nd
                for j = length(obj.A) + 1 : nd
                    obj.A{j} = [];
                end
            end
            if length(obj.B) < nd
                for j = length(obj.B) + 1 : nd
                    obj.B{j} = [];
                end
            end
            if length(obj.f) < nd
                for j = length(obj.f) + 1 : nd
                    obj.f{j} = [];
                end
            end
            if length(obj.C) < nd
                for j = length(obj.C) + 1 : nd
                    obj.C{j} = [];
                end
            end
            if length(obj.D) < nd
                for j = length(obj.D) + 1 : nd
                    obj.D{j} = [];
                end
            end
            if length(obj.e) < nd
                for j = length(obj.e) + 1 : nd
                    obj.e{j} = [];
                end
            end
            
            % fill in cells using previous value and check consistency
            for i = 1 : nd
                
                if ( i > 1 )
                    
                    if isempty(obj.A{i})
                        obj.A{i} = obj.A{i-1};
                    end
                    
                    if isempty(obj.B{i})
                        obj.B{i} = obj.B{i-1};
                    end
                    
                    if isempty(obj.C{i})
                        obj.C{i} = obj.C{i-1};
                    end
                    
                    if isempty(obj.D{i})
                        obj.D{i} = obj.D{i-1};
                    end
                    
                    if isempty(obj.f{i})
                        obj.f{i} = obj.f{i-1};
                    end
                    
                    if isempty(obj.e{i})
                        obj.e{i} = obj.e{i-1};
                    end
                    
                end
                
                if size(obj.A{i},1) ~= size(obj.A{i},2)
                    error(['A[' num2str(i) '] is not a square matrix']);
                end
                
                if size(obj.B{i},1)~= size(obj.A{i},1)
                    error(['Dimension mismatch between A[' num2str(i) '] and B[' num2str(i) '] system matrices']);
                end
                
                if size(obj.C{i},1)~= size(obj.D{i},1)
                    error(['Dimension mismatch between C[' num2str(i) '] and D[' num2str(i) '] system matrices']);
                end
                
                if size(obj.C{i},2)~= size(obj.A{i},2)
                    error(['Dimension mismatch between A[' num2str(i) '] and C[' num2str(i) '] system matrices']);
                end
                
                if size(obj.D{i},2)~= size(obj.B{i},2)
                    error(['Dimension mismatch between B[' num2str(i) '] and D[' num2str(i) '] system matrices']);
                end
                
                if size(obj.f{i},1)~= size(obj.A{i},1)
                    error(['Dimension mismatch between A[' num2str(i) '] and f[' num2str(i) '] system matrices']);
                end
                
                if size(obj.f{i},2)~= 1
                    error(['f[' num2str(i) '] is not a column vector']);
                end
                
                if size(obj.e{i},1)~= size(obj.C{i},1)
                    error(['Dimension mismatch between C[' num2str(i) '] and e[' num2str(i) '] system matrices']);
                end
                
                if size(obj.e{i},2)~= 1
                    error(['e[' num2str(i) '] is not a column vector']);
                end
                
            end
            
            
        end
        
        
        function obj = fillCost( obj )
        % fill TV cost
            
            % convert all parameters to cells
            if ~iscell(obj.Q)
                obj.Q = {obj.Q};
            end
            if ~iscell(obj.R)
                obj.R = {obj.R};
            end
            if ~iscell(obj.S)
                obj.S = {obj.S};
            end
            if ~iscell(obj.gy)
                obj.gy = {obj.gy};
            end
            if ~iscell(obj.gu)
                obj.gu = {obj.gu};
            end
            
            nd = max([length(obj.Q) length(obj.R) length(obj.S) ...
                length(obj.gy) length(obj.gu) obj.ni]);
            
            % extend cells with empty instances
            if length(obj.Q) < nd
                for j = length(obj.Q) + 1 : nd
                    obj.Q{j} = [];
                end
            end
            if length(obj.R) < nd
                for j = length(obj.R) + 1 : nd
                    obj.R{j} = [];
                end
            end
            if length(obj.S) < nd
                for j = length(obj.S) + 1 : nd
                    obj.S{j} = [];
                end
            end
            if length(obj.gy) < nd
                for j = length(obj.gy) + 1 : nd
                    obj.gy{j} = [];
                end
            end
            if length(obj.gu) < nd
                for j = length(obj.gu) + 1 : nd
                    obj.gu{j} = [];
                end
            end
            
            % fill in cells using previous value and check consistency
            for i = 1 : nd
                
                if i > 1
                    
                    if isempty(obj.Q{i})
                        obj.Q{i} = obj.Q{i-1};
                    end
                    
                    if isempty(obj.R{i})
                        obj.R{i} = obj.R{i-1};
                    end
                    
                    if isempty(obj.S{i})
                        obj.S{i} = obj.S{i-1};
                    end
                    
                    if isempty(obj.gy{i})
                        obj.gy{i} = obj.gy{i-1};
                    end
                    
                    if isempty(obj.gu{i})
                        obj.gu{i} = obj.gu{i-1};
                    end
                    
                end
                
                if size(obj.Q{i},1) ~= size(obj.Q{i},2) || any(size(obj.Q{i})~= obj.ny)
                    error(['Dimension mismatch in Q[' num2str(i) '] matrix']);
                end
                
                if size(obj.R{i},1) ~= size(obj.R{i},2) || any(size(obj.R{i})~= obj.nu)
                    error(['Dimension mismatch in R[' num2str(i) '] matrix']);
                end
                
                if size(obj.S{i},1) ~=  obj.ny || size(obj.S{i},2) ~=  obj.nu
                    error(['Dimension mismatch in S[' num2str(i) '] matrix']);
                end
                
                if size(obj.gy{i},1) ~= obj.ny || size(obj.gy{i},2) ~= 1
                    error(['Dimension mismatch in gy[' num2str(i) '] vector']);
                end
                
                if size(obj.gu{i},1) ~= obj.nu || size(obj.gu{i},2) ~= 1
                    error(['Dimension mismatch in gu[' num2str(i) '] vector']);
                end
                
                if min(eig(obj.Q{i})) < 0
                    error(['Q[' num2str(i) '] matrix is not positive semi-definite']);
                end
                if min(eig(obj.R{i})) < 0
                    error(['R[' num2str(i) '] matrix is not positive semi-definite']);
                end
                
            end
                        
        end
       
        
        function obj = fillConstraints( obj )
        % fill TV constraints
        
            % convert all parameters to cells
            if ~iscell(obj.ymin)
                obj.ymin = {obj.ymin};
            end
            if ~iscell(obj.ymax)
                obj.ymax = {obj.ymax};
            end
            if ~iscell(obj.umin)
                obj.umin = {obj.umin};
            end
            if ~iscell(obj.umax)
                obj.umax = {obj.umax};
            end
            if ~iscell(obj.dmin)
                obj.dmin = {obj.dmin};
            end
            if ~iscell(obj.dmax)
                obj.dmax = {obj.dmax};
            end
            if ~iscell(obj.M)
                obj.M = {obj.M};
            end
            if ~iscell(obj.N)
                obj.N = {obj.N};
            end
        
            nd = max([length(obj.ymin) length(obj.ymax) length(obj.umin) ...
                length(obj.umax) length(obj.dmin) length(obj.dmax) ...
                length(obj.M) length(obj.N) obj.ni]);
            
             % extend cells with empty instances
             if length(obj.ymin) < nd
                 for j = length(obj.ymin) + 1 : nd
                     obj.ymin{j} = [];
                 end
             end
             if length(obj.ymax) < nd
                 for j = length(obj.ymax) + 1 : nd
                     obj.ymax{j} = [];
                 end
             end
             if length(obj.umin) < nd
                 for j = length(obj.umin) + 1 : nd
                     obj.umin{j} = [];
                 end
             end
             if length(obj.umax) < nd
                 for j = length(obj.umax) + 1 : nd
                     obj.umax{j} = [];
                 end
             end
             if length(obj.dmin) < nd
                 for j = length(obj.dmin) + 1 : nd
                     obj.dmin{j} = [];
                 end
             end
             if length(obj.dmax) < nd
                 for j = length(obj.dmax) + 1 : nd
                     obj.dmax{j} = [];
                 end
             end
             if length(obj.M) < nd
                 for j = length(obj.M) + 1 : nd
                     obj.M{j} = [];
                 end
             end
             if length(obj.N) < nd
                 for j = length(obj.N) + 1 : nd
                     obj.N{j} = [];
                 end
             end
             
            % fill in cells using previous value and check consistency
             for i = 2 : nd
                 
                 if isempty(obj.M{i})
                     obj.M{i} = obj.M{i-1};
                 end
                 
                 if isempty(obj.N{i})
                     obj.N{i} = obj.N{i-1};
                 end
                 
                 if isempty(obj.dmax{i})
                     obj.dmax{i} = obj.dmax{i-1};
                 end
                 
                 if isempty(obj.dmin{i})
                     obj.dmin{i} = obj.dmin{i-1};
                 end
                 
                 if isempty(obj.ymax{i})
                     obj.ymax{i} = obj.ymax{i-1};
                 end
                 
                 if isempty(obj.ymin{i})
                     obj.ymin{i} = obj.ymin{i-1};
                 end
                 
                 if isempty(obj.umax{i})
                     obj.umax{i} = obj.umax{i-1};
                 end
                 
                 if isempty(obj.umin{i})
                     obj.umin{i} = obj.umin{i-1};
                 end
                 
             end
             
        end

        
        function obj = fillTrajectories( obj )
        % fill TV trajectories
        
            % convert all parameters to cells
            if ~iscell(obj.yr)
                obj.yr = {obj.yr};
            end
            if ~iscell(obj.ur)
                obj.ur = {obj.ur};
            end
            
            % extend cells with empty instances
            nd = max([length(obj.yr) length(obj.ur) obj.ni]);
            
            if length(obj.yr) < nd
                for j = length(obj.yr) + 1 : nd
                    obj.yr{j} = [];
                end
            end
            if length(obj.ur) < nd
                for j = length(obj.ur) + 1 : nd
                    obj.ur{j} = [];
                end
            end
            
            % fill in cells using previous value and check consistency
            for i = 1 : nd
                
                if i>1
                    
                    if isempty(obj.yr{i})
                        obj.yr{i} = obj.yr{i-1};
                    end      
                    if isempty(obj.ur{i})
                        obj.ur{i} = obj.ur{i-1};
                    end
                    
                end
                
                if length(obj.yr{i}) ~= obj.ny || ...
                        length(obj.ur{i}) ~= obj.nu
                    error('Inconsistent dimension of reference signal')
                end
                
            end
            
        end
        
        
        function obj = trimBenchmark( obj,n )
        % trim benchmark data bigger than n
        
            if iscell(obj.A) && length(obj.A) > n
                obj.A(n+1:end) = [];
                obj.B(n+1:end) = [];
                obj.f(n:end)   = [];
                obj.C(n+1:end) = [];
                obj.D(n+1:end) = [];
                obj.e(n+1:end) = [];
            end
            
            if iscell(obj.yr) && length(obj.yr) > n
                obj.yr(n+1:end) = [];
            end
            
            if iscell(obj.ur) && length(obj.ur) > n
                obj.ur(n+1:end) = [];
            end
            
            if iscell(obj.Q) && length(obj.Q) > n
                obj.Q(n+1:end)    = [];
                obj.R(n+1:end)    = [];
                obj.S(n:end)      = [];
                obj.gu(n+1:end)   = [];
                obj.gy(n+1:end)   = [];
            end
            
            if iscell(obj.ymin) && length(obj.ymin) > n
                obj.M(n+1:end)    = [];
                obj.N(n+1:end)    = [];
                obj.dmin(n+1:end) = [];
                obj.dmax(n+1:end) = [];
                obj.ymin(n+1:end) = [];
                obj.ymax(n+1:end) = [];
                obj.umin(n+1:end) = [];
                obj.umax(n+1:end) = [];
            end
            
            
        end

    end
    
end
