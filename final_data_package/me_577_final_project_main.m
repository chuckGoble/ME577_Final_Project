% Main script for ME 577 Group Final Project
close all; clear; clc;

%% User Inputs
% Initial Conditions
ICs = [0; 0; deg2rad(0); 0];

% Controller Design Parameter: Percent Overshoot
desPO = 10;

% Controller Design Parameter: Settling Time
tSettle = 1;

% Controller Design Parameter: Steady State Metric
inputPercent = 2;

% Controller Design Parameter: Pole Placement Scalar Multiplier
poleScalarOne = 5;
poleScalarTwo = 10;

% Sim Time Vector
timeVec = 0:1E-3:100;

%% System Response Parameters
% Pole Placement and LQR Knobs
pertTime = 1;
stepAmp = 100;
nonUnitImpulseAmp = 100;
sineAmp = 100;
sineFreqLow = 100;
sineFreqHigh = 1000;

%% System Definition -- Constants
m = 0.35;
M = 2.2;
L = 1.3;
b = 0.25;
g = 9.8;

shift = [0, 0, pi/2, 0];

%% Linear EOM Simulation - Newtonian Dynamics Model
A = zeros(4);
A(1,2) = 1;
A(2,2) = (b)./((2*m)+M);
A(2,3) = (m*g)./((2*m)+M);
A(3,4) = 1;
A(4,2) = (-1.*b)./(L.*((2*m)+M));
A(4,3) = (g./L) - ((m*g)./((2*m)+M));

% B Matrix
B = zeros(4,1);
B(2,1) = (1)./((2*m)+M);
B(4,1) = (-1)./(L.*((2*m)+M));

% C Matrix
C = eye(4);

% D Matrix
D = zeros(4, 1);

% % Create a Matlab State Space Model
linSys = ss(A,B,C,D);

%%  Initial Conditions Simulation
[yInitial, tInitial] = initial(linSys, ICs, timeVec);
yInitial = moveOutput(yInitial, shift);
plotStates(yInitial, tInitial, 'Uncontrolled Initial Conditions Simulation');

%%  Unit Step Input Simulation
[yStep, tStep] = step(linSys, timeVec);
yStep = moveOutput(yStep, shift);
plotStates(yStep, tStep, 'Uncontrolled Unit Step Simulation');

%%  Unit Impulse Input Simulation
[yImpulse, tImpulse] = impulse(linSys, timeVec);
yImpulse = moveOutput(yImpulse, shift);
plotStates(yImpulse, tImpulse, 'Uncontrolled Unit Impulse Simulation');

%% Stability -> HW 4 type of analysis
[eigVectors, eigValues] = eig(A);
linSysPole = pole(linSys);

%% Controlability -> HW 5 type of analysis
P = [B, A*B, A*A*B, A*A*A*B];
rankOfP = rank(P);

%% Observability -> HW 5 type of analysis
Q = [C;...
    C*A;...
    C*A*A;...
    C*A*A*A];
rankOfQ = rank(Q);

%% Pole Placement
zeta = (-1*log(desPO./100))/(sqrt((pi.^2) + (desPO./100)));
wn = (-1*log(inputPercent./100))./(tSettle.*zeta);
eqn = [1, 2.1, 3.4, 2.7.*wn, wn.^2];
p = roots(eqn);
stableP = p(real(p) == min(real(p)));
p = [stableP(1); stableP(2);...
    floor(poleScalarOne*min(real(stableP)));...
    floor(poleScalarTwo*min(real(stableP)))];

[K, prec] = place(A, B, p);
closeLoopA = A - (B*K);
closeLoopSysPolePlace = ss(closeLoopA, B, C, D);

%% Pole Placement Controller Design Plots
% Unit Step Input Simulation
[yStepPole, tStepPole] = step(closeLoopSysPolePlace, timeVec);
yStepPole = moveOutput(yStepPole, shift);
plotStates(yStepPole, tStepPole, 'Pole Placement Controller Unit Step Simulation');

%  Unit Impulse Input Simulation
[yImpulsePole, tImpulsePole] = impulse(closeLoopSysPolePlace, timeVec);
yImpulsePole = moveOutput(yImpulsePole, shift);
plotStates(yImpulsePole, tImpulsePole, 'Pole Placement Controller Unit Impulse Simulation');

%% Simulink -- Integral Error Control
% Define new A and B matrices
cIEC = [1 0 0 0];
Ahat = [A zeros(length(A),1); -cIEC 0];
Bhat = [B; 0];

% Check controllabilty of new system. Rank = 5 means that thes system
% is % fully state controllable.
P = [A, B; -cIEC, 0];
rank(P);

% Define desired eigenvalues for new 5th order system. Eigenvalues were
% defined from values that result in a 1 second settling time
eigDesired = [-6.4480,-4.1104+6.3142i,-4.1104-6.3142i,...
    -5.9268+3.0813i,-5.9268-3.0813i];

% Use place command to calculate gain matrix to acheive desired poles
gainMatrix = place(Ahat,Bhat,eigDesired);

% State feedback gain matrix
K = gainMatrix(1:4);

% Integral error tracking gain value
KI = -gainMatrix(5);

% Define desired cart position
cartPosition = 5;

% Define periodic input disturbance amplitude and frequency
periodicDisturbanceAmp = 100;
periodicDisturbanceFreq = 10;

% Define impulse input disturbance amplitude
impulseAmp = 0;

% Run Simulink model
out = sim('stateFeedbackIntegralControl.slx');
intErrPlot(out);

%% LQR Controller
lqr_q_mat = zeros(4);

% Grey Wolf Optimizer
lqr_q_mat(1, 1) = 3.471E6;
lqr_q_mat(2, 2) = 4.91E3;
lqr_q_mat(3, 3) = 3.54E-4;
lqr_q_mat(4, 4) = 1.11E4;

lqr_r = 8779.759;

[lqr_gain, lqr_s, lqr_poles] = lqr(A, B, lqr_q_mat, lqr_r);

closeLoopSysLqr = ss(A-B*lqr_gain, B, C, D);

%% LQR Controller Design Plots
[yStepLqr, tStepLqr] = step(closeLoopSysLqr, timeVec);
yStepLqr = moveOutput(yStepLqr, shift);
plotStates(yStepLqr, tStepLqr, 'LQR Controller Unit Step Simulation');

[yImpulseLqr, tImpulseLqr] = impulse(closeLoopSysLqr, timeVec);
yImpulseLqr = moveOutput(yImpulseLqr, shift);
plotStates(yImpulseLqr, tImpulseLqr, 'LQR Controller Unit Impulse Simulation');

%% System Response Analysis
% Non Unit Impulse
nonUnitImp = genImpulseInput(timeVec, nonUnitImpulseAmp, pertTime);
[yImpPole, tImpPole] = lsim(closeLoopSysPolePlace, nonUnitImp, timeVec);
yImpPole = moveOutput(yImpPole, shift);
[yImpLqr, tImpLqr] = lsim(closeLoopSysLqr, nonUnitImp, timeVec);
yImpLqr = moveOutput(yImpLqr, shift);

% Non Unit Step
nonUnitStep = genStepInput(timeVec, stepAmp, pertTime);
[yStepPole, tStepPole] = lsim(closeLoopSysPolePlace, nonUnitStep, timeVec);
yStepPole = moveOutput(yStepPole, shift);
[yStepLqr, tStepLqr] = lsim(closeLoopSysLqr, nonUnitStep, timeVec);
yStepLqr = moveOutput(yStepLqr, shift);

% Ramp
ramp = genRampInput(timeVec, pertTime);
[yRampPole, tRampPole] = lsim(closeLoopSysPolePlace, ramp, timeVec);
yRampPole = moveOutput(yRampPole, shift);
[yRampLqr, tRampLqr] = lsim(closeLoopSysLqr, ramp, timeVec);
yRampLqr = moveOutput(yRampLqr, shift);

% low freq sine
lowFreqSine = genSinusodialInput(timeVec, pertTime, sineAmp, sineFreqLow);
[ySinePole, tSinePole] = lsim(closeLoopSysPolePlace, lowFreqSine, timeVec);
ySinePole = moveOutput(ySinePole, shift);
[ySineLqr, tSineLqr] = lsim(closeLoopSysLqr, lowFreqSine, timeVec);
ySineLqr = moveOutput(ySineLqr, shift);

% high freq sine
highFreqSine = genSinusodialInput(timeVec, pertTime, sineAmp, sineFreqHigh);
[ySinePole2, tSinePole2] = lsim(closeLoopSysPolePlace, highFreqSine, timeVec);
ySinePole2 = moveOutput(ySinePole2, shift);
[ySineLqr2, tSineLqr2] = lsim(closeLoopSysLqr, highFreqSine, timeVec);
ySineLqr2 = moveOutput(ySineLqr2, shift);

%% System Response Analysis Plots
genMultiPlot(tImpPole, yImpPole, 'Pole Placement',...
    tImpLqr, yImpLqr, 'LQR', 'Non-Unit Impulse Disturbance Simulation')
genMultiPlot(tStepPole, yStepPole, 'Pole Placement',...
    tStepLqr, yStepLqr, 'LQR', 'Non-Unit Step Disturbance Simulation')
genMultiPlot(tRampPole, yRampPole, 'Pole Placement',...
    tRampLqr, yRampLqr, 'LQR', 'Ramp Disturbance Simulation')
genMultiPlot(tSinePole, ySinePole, 'Pole Placement',...
    tSineLqr, ySineLqr, 'LQR', 'Low Frequency Sine Disturbance Simulation')
genMultiPlot(tSinePole2, ySinePole2, 'Pole Placement',...
    tSineLqr2, ySineLqr2, 'LQR', 'High Frequency Sine Disturbance Simulation')

%% Helper Functions
function [y] = moveOutput(y, shift)
    for ii = 1:1:size(y, 1)
        y(ii, :) = y(ii, :) + shift;
    end
end

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

function [u] = genStepInput(tVec, Amp, stepTime)
    
    u = NaN(size(tVec));

    for ii = 1:1:length(u)
        if tVec(ii) > stepTime
            u(ii) = Amp;
        else
            u(ii) = 0;
        end
    end
end

function [u] = genImpulseInput(tVec, Amp, impulseTime)
    u = NaN(size(tVec));

    for ii = 1:1:length(u)
        if tVec(ii) == impulseTime
            u(ii) = Amp;
        else
            u(ii) = 0;
        end
    end
end

function [u] = genRampInput(tVec, timeShift)
    u = tVec - timeShift;
end

function [u] = genSinusodialInput(tVec, timeShift, amp, freq)
    
    omega = freq .* 2 .* pi;

    u = NaN(size(tVec));

    for ii = 1:1:length(u)
        if tVec(ii) > timeShift
            u(ii) = amp.*sin(omega.*(tVec(ii)-timeShift));
        else
            u(ii) = 0;
        end
    end
end

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