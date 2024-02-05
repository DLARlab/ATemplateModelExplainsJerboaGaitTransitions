%% Function that build up the 1-D continuation problem
function [xFINAL,fval] = ContinuationEqn(X0,x_Last,Offsets,radius,numOPTS)
% Regulate Event Timing
X0 = Timingcheck(X0);
% Find solution X zero in residual values
[xFINAL,fval] = fsolve(@Residual15, X0, numOPTS);


function residual = Residual15(X)
    
    % Build new residual values zero in fsolve
    [residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset(X,Offsets);
    % Constrain that limit the distance of new solution from current one
    residual(end+1) = norm(X(1:7)-x_Last(1:7))-radius;
end

end

%% Function: Event Timing Regulation (Outside Zero Function)
function X_out = Timingcheck(X_in)
    X_out = X_in;
    % Event timing:
    tL_TD    = X_in(8);
    tL_LO    = X_in(9);
    tR_TD    = X_in(10);
    tR_LO    = X_in(11);
    tAPEX    = X_in(12);
    % Move all timing values into [0..tR_TD]
    while tL_TD < 0
         tL_TD = tL_TD + tAPEX;
    end
    while tL_TD > tAPEX
         tL_TD = tL_TD - tAPEX;
    end 
    while tL_LO < 0
         tL_LO = tL_LO + tAPEX;
    end
    while tL_LO > tAPEX
         tL_LO = tL_LO - tAPEX;
    end 
    while tR_TD < 0
         tR_TD = tR_TD + tAPEX;
    end
    while tR_TD > tAPEX
         tR_TD = tR_TD - tAPEX;
    end 
    while tR_LO < 0
         tR_LO = tR_LO + tAPEX;
    end
    while tR_LO > tAPEX
         tR_LO = tR_LO - tAPEX;
    end
    X_out(8:12) = [tL_TD; tL_LO; tR_TD; tR_LO; tAPEX ];
end
