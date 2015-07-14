function [ squareWave ] = generateSquareWave( PeriodLength,nPeriods,MaxValue, MinValue,r )
%generateSquareWave generates a square wave reference.
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


    squareWave = {[]};
    squareWave{1} = [MaxValue/r 0 MaxValue 0]';
    
    for k = 1:nPeriods
        for i = 1:ceil(PeriodLength/2)
            squareWave{end + 1} = [MaxValue/r 0 MaxValue 0]';
        end

        for i = ceil(PeriodLength/2)+ 1:PeriodLength
            squareWave{end + 1} = [MinValue/r 0 MinValue 0]';
        end
    end

end
