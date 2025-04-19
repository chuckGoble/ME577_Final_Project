function [] = plotStates(y, t, titleString)
    figure()
    subplot(4, 1, 1)
    plot(t, y(:, 1))
    grid on
    xlabel('Time [sec]')
    ylabel('Displacement [m]')
    title(titleString)
    
    subplot(4, 1, 2)
    plot(t, y(:, 2))
    grid on
    xlabel('Time [sec]')
    ylabel('Displacement Rate [m/s]')
    
    subplot(4, 1, 3)
    plot(t, rad2deg(y(:, 3)))
    grid on
    xlabel('Time [sec]')
    ylabel('Angle [deg]')
    
    subplot(4, 1, 4)
    plot(t, rad2deg(y(:, 4)))
    grid on
    xlabel('Time [sec]')
    ylabel('Angle Rate [deg/s]')
end