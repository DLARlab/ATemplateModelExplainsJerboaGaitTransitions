%% Function would be able to scan the coresponding solution branch with given parameter.
% Self-tunning radius function if embedded, one can enable it inside this fucntion.
function results = NumericalContinuation1D(X1,X2,Offset,radius,numOPTS,iterations)
    % Define the solution matrix. Search negative direction first.
    results(:,1) = [X1;Offset]; 
    results(:,2) = [X2;Offset];
    results = flip(sortrows(results.',1).',2);% sort so that result2 < result1
%     radius = norm(X1-X2);
%    radius = 0.02;
    cs = 0; % current size of solution matrix
    
    % Parameter in charge of self-tuning step size. 
    % Basically the 
    SelfTuningSetpSize = 0;
    MaxSize = 0.2;
    
     
    % Loop that search both positive & negative direction of 1-D manifold.
    for j = 1:2
        % Initialize xCYC & x_Last for inner loop.
        xCYC = results(1:12,end);
        x_Last = xCYC;
        
        % Loop that search along one direction of 1-D manifold. Using average step method.
        for k = 1:iterations
            
            % Intenal intergrated function that would be able to tune step size. Call when enabled.
            if SelfTuningSetpSize==1
                radius = StepSizeTuning(results,MaxSize);
            end
            
            % Initial guess in this iteration.(Average step method)
            X0 = xCYC + (results(1:12,k+cs+1)-results(1:12,k+cs));
            if X0(8)>X0(12) || X0(8)<0
               X0(1) = X0(1) + 3*(results(1,k+cs+1)-results(1,k+cs));
            end
            X0 = Timingcheck(X0);

            % Find the solution for this iteration
                for TryTime = 1:2
                    [xFINAL,fval] = ContinuationEqn(X0,x_Last,Offset,radius,numOPTS);
                    if norm(fval) < 1e-9
                    % This is a periodic solution
                        break;
                    end 
                    % Otherwise, try a again   
                    X0 = xCYC + (results(1:12,k+cs+1)-results(1:12,k+cs))*((rand-0.5)*1e-2 + 1);
                end
                             
            % End searching for this direction if cannot find such solution.    
                if TryTime == 2
                    disp('Can not find a solution within 2 trials.')
                    break;
                end
                
            % Solution that have vertical speed < 0 has no physical meaning    
                if  xFINAL(1)<0.01
                    disp('Algorithm stopped because solution cross zero.')
                    break;
                end
               
          xCYC = xFINAL;  
          x_Last = xCYC;
          results(:,k+cs+2) = [xCYC;Offset]; 
          disp(norm(fval))
          
          % Check if solution branch merge in circle, if so, there is no need to keep searching.     
          dir = sign(results(1,2+cs)-results(1,1+cs));
                if sign(results(1,k+cs+2)-results(1,k+cs+1))==dir && norm(xFINAL-results(1:12,1)) < norm(results(1:12,2)-results(1:12,1)) && dir*xFINAL(1)>dir*results(1,1)
                    disp('Solution Circle Found.')
                    return;
                end
        end
        % Flip the results matrix(sort ascendingly), prepare for searching in positive direction.
        if j == 1
        results = flip(results,2);
        cs = size(results,2)-2;
        end
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

%% Intenal intergrated function that would be able to tune step size. Call when enabled.
function radius = StepSizeTuning(results,MaxSize)

    if size(results,2)==2
        radius = MaxSize;
    else
        % Get the angle between last two steps.
        step1 = results(1:12,end)-results(1:12,end-1);
        step2 = results(1:12,end-1)-results(1:12,end-2);
        sigma = abs( dot(step1,step2)/(norm(step1)*norm(step2)) );
        % Adjust the radius according to the sharpness of turning. 
        radius = sigma*MaxSize;
    end

end