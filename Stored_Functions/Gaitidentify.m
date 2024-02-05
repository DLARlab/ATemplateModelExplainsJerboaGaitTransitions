function gait = Gaitidentify(X)
    if X(9) < X(8) && X(11) < X(10)
        walk = 1; % walking
        gait = 0;
        return
    else
        walk = 0;
    end

    threshold = 1e-6; % threshold for identifying hopping
    if ~walk && X(9)-X(11) > threshold && X(8)-X(10) > threshold
        leftleading = 1;
        rightleading = 0;
    elseif ~walk && X(11)-X(9) > threshold && X(10)-X(8) > threshold
        leftleading =0;
        rightleading = 1;
    else % hopping
        gait = 1;
        return
    end

    if leftleading && X(8)<=X(11) && X(10)<=X(9)
        gait = 2; % galloping
    elseif rightleading && X(10)<=X(9) && X(8)<=X(11)
        gait = 2; % galloping
    elseif leftleading &&  X(8)>X(11) && X(10)<=X(9)
        gait = 3; % running
    elseif rightleading && X(10)>X(9) && X(8)<=X(11)
        gait = 3; % running
    end
end