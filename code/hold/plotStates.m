function [] = plotStates(y, t, titleString)
    figure()
    subplot(4, 1, 1)
    plot(t, y(:, 1), 'Linewidth', 3)
    grid on
    xlabel('Time [sec]', 'fontweight', 'bold', 'fontsize', 14)
    ylabel('Displacement [m]', 'fontweight', 'bold', 'fontsize', 14)
    title(titleString, 'fontweight', 'bold', 'fontsize', 14)
    a = get(gca,'XTickLabel');
    set(gca,'XTickLabel', a,'fontsize', 14)
    
    subplot(4, 1, 2)
    plot(t, y(:, 2), 'Linewidth', 3)
    grid on
    xlabel('Time [sec]', 'fontweight', 'bold', 'fontsize', 14)
    ylabel('Displacement Rate [m/s]', 'fontweight', 'bold', 'fontsize', 14)
    a = get(gca,'XTickLabel');
    set(gca,'XTickLabel', a,'fontsize', 14)

    subplot(4, 1, 3)
    plot(t, rad2deg(y(:, 3)), 'Linewidth', 3)
    grid on
    xlabel('Time [sec]', 'fontweight', 'bold', 'fontsize', 14)
    ylabel('Angle [deg]', 'fontweight', 'bold', 'fontsize', 14)
    a = get(gca,'XTickLabel');
    set(gca,'XTickLabel', a,'fontsize', 14)
    
    subplot(4, 1, 4)
    plot(t, rad2deg(y(:, 4)), 'Linewidth', 3)
    grid on
    xlabel('Time [sec]', 'fontweight', 'bold', 'fontsize', 14)
    ylabel('Angle Rate [deg/s]', 'fontweight', 'bold', 'fontsize', 14)
    a = get(gca,'XTickLabel');
    set(gca,'XTickLabel', a,'fontsize', 14)
end