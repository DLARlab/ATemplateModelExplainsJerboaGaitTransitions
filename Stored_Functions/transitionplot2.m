function transitionplot2(results,color_plot,linewidth,brightness)

    startpoint = 1;
    endpoint = 1;
    
    gaitp = Gaitidentify(results(:,1));
    for i = 1:size(results,2)
        gaitc = Gaitidentify(results(:,i));
        switch gaitp
           case 0
               linestyle = '--o';
           case 1
               % linestyle = '--';
               linestyle = '-';
           case 2
               linestyle = '-';
           case 3
               linestyle = ':';
               % linestyle = '-';
               % color_plot = brighten([0.4660 0.6740 0.1880],brightness);
        end
        
        
        % Plot line segment with respect of gait pattern, trigger when different gait detected
        if gaitc==gaitp
            endpoint = i;
            if endpoint == size(results,2)
                hold on
                plot3(results(1,startpoint:endpoint),results(4,startpoint:endpoint),results(2,startpoint:endpoint),linestyle,'LineWidth',linewidth,'color',color_plot)
            end
        else
            hold on
            plot3(results(1,startpoint:endpoint),results(4,startpoint:endpoint),results(2,startpoint:endpoint),linestyle,'LineWidth',linewidth,'color',color_plot)
            startpoint = i;
        end
        gaitp = gaitc;
    end
end
