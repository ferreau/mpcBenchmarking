classdef Benchmarks < uint32
%Benchmarks enumeration summarizing the names of all MPC benchmark examples.

% This file is part of the CAESAR MPC Suite developed at 
% ABB Corporate Research (CHCRC.C1).
% It is distributed under the terms of the Eclipse Public License v1.0,
% see the file LICENSE in the root directory.
%
% Authors:         Joachim Ferreau, Helfried Peyrl, 
%                  Dimitris Kouzoupis, Andrea Zanelli
% Last modified:   14/7/2015


    enumeration
        ... % part of public benchmark collection
        aircraft                 (1)
        ballOnPlate              (2)
        dcMotor                  (3)
        toyExample1              (4)
        forcesExample            (5)
        helicopter               (6)
        nonlinearCstr            (7)
        pendulum                 (8)
        quadcopter               (9)
        shell                    (10)
        spacecraft               (11)
        springMass               (12)
        binaryDistillationColumn (13)
        doubleInvertedPendulum   (14)
        tripleInvertedPendulum   (15)
        robotArm                 (16)
        nonlinearChain           (17)
        fiordosExample           (18)
        overheadCrane            (19)
        polytopicTerminal        (20)
        ... % industrial, not part of public benchmark collection
        compressor               (21)
        ... % under development
        nonlinearAcDrive         (22)
        %powerNetworkSystem      (xx)
        %underwaterVehicle       (xx)
        %dieselEngine            (xx)
        %linearizedCstr          (xx)
    end
    
end
