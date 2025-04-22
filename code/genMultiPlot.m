function [] = genMultiPlot(t1, y1, legend_1, t2, y2, legend_2, titleString)
    figure()
    subplot(4, 1, 1)
    hold on
    plot(t1, y1(:, 1), 'Linewidth', 3)
    plot(t2, y2(:, 1), 'Linewidth', 3)
    grid on
    legend(legend_1, legend_2, 'Location', 'BestOutside')
    xlabel('Time [sec]', 'fontweight', 'bold', 'fontsize', 14)
    ylabel('Displacement [m]', 'fontweight', 'bold', 'fontsize', 14)
    title(titleString, 'fontweight', 'bold', 'fontsize', 14)
    
    subplot(4, 1, 2)
    hold on
    plot(t1, y1(:, 2), 'Linewidth', 3)
    plot(t2, y2(:, 2), 'Linewidth', 3)
    hold off
    grid on
    legend(legend_1, legend_2, 'Location', 'BestOutside')
    xlabel('Time [sec]', 'fontweight', 'bold', 'fontsize', 14)
    ylabel('Displacement Rate [m/s]', 'fontweight', 'bold', 'fontsize', 14)

    subplot(4, 1, 3)
    hold on
    plot(t1, rad2deg(y1(:, 3)), 'Linewidth', 3)
    plot(t2, rad2deg(y2(:, 3)), 'Linewidth', 3)
    hold off
    grid on
    legend(legend_1, legend_2, 'Location', 'BestOutside')
    xlabel('Time [sec]', 'fontweight', 'bold', 'fontsize', 14)
    ylabel('Angle [deg]', 'fontweight', 'bold', 'fontsize', 14)
    
    subplot(4, 1, 4)
    hold on
    plot(t1, rad2deg(y1(:, 4)), 'Linewidth', 3)
    plot(t2, rad2deg(y2(:, 4)), 'Linewidth', 3)
    hold off
    grid on
    legend(legend_1, legend_2, 'Location', 'BestOutside')
    xlabel('Time [sec]', 'fontweight', 'bold', 'fontsize', 14)
    ylabel('Angle Rate [deg/s]', 'fontweight', 'bold', 'fontsize', 14)
end