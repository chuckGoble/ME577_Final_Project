function [] = intErrPlot(out)
    figure()
    subplot(2, 1, 1)
    hold on
    plot(out.tout,out.command,'--k', 'Linewidth', 3)
    plot(out.tout,out.position,'-r', 'Linewidth', 3)
    hold off
    grid on
    legend("Commanded","Cart", 'fontweight', 'bold', 'fontsize', 14, 'Location',...
        'BestOutside')
    xlabel('Time [sec]', 'fontweight', 'bold', 'fontsize', 14)
    ylabel('Displacement [m]', 'fontweight', 'bold', 'fontsize', 14)
    a = get(gca,'XTickLabel');
    set(gca,'XTickLabel', a,'fontsize', 14)
    
    subplot(2, 1, 2)
    hold on
    plot(out.tout, 90.*ones(size(out.theta, 1), 1),'--k', 'Linewidth', 3)
    plot(out.tout, rad2deg(out.theta + pi./2),'r', 'Linewidth', 3)
    hold off
    grid on
    legend("Equilibrium","Cart", 'fontweight', 'bold', 'fontsize', 14, 'Location',...
        'BestOutside')
    xlabel('Time [sec]', 'fontweight', 'bold', 'fontsize', 14)
    ylabel('Angle [deg]', 'fontweight', 'bold', 'fontsize', 14)
    a = get(gca,'XTickLabel');
    set(gca,'XTickLabel', a,'fontsize', 14)
end