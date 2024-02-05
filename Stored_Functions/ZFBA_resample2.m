function [xr,yr,alphaLr,alphaRr] = ZFBA_resample2(T,Y,To)

    Y1 = Y(:,1);
    Y3 = Y(:,3);
    Y5 = Y(:,5);
    Y7 = Y(:,7);
    % resample values with respect to To instead of T
    [~, ind] = unique(Y1);
    xr = interp1(T(ind),Y1(ind),To,'linear','extrap');
    [~, ind] = unique(Y3);
    yr = interp1(T(ind),Y3(ind),To,'linear','extrap');
    [~, ind] = unique(Y5);
    alphaLr = interp1(T(ind),Y5(ind),To,'linear','extrap');
    [~, ind] = unique(Y7);
    alphaRr = interp1(T(ind),Y7(ind),To,'linear','extrap');

end