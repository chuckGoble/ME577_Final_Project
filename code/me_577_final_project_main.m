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
timeVec = 0:1E-3:3;

% System Response Parameters
pertTime = 1;
stepAmp = 100;
nonUnitImpulseAmp = 100;

%% System Definition -- Constants
m = 0.35;
M = 2.2;
L = 1.3;
b = 0.25;
g = 9.8;

shift = [0, 0, pi/2, 0];

%% NonLinear EOM Simulation - Newtonian Dynamics Model
% TODO: Add nonlinear ode45 simulation
% TODO: Show that the nonlinear system is not stable such that it goes
% unstable due to floating point error integration error.

%% Linear EOM Simulation - Newtonian Dynamics Model
% New Dynamics Model
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
%plotStates(yInitial, tInitial, 'Uncontrolled Initial Conditions Simulation');

%%  Unit Step Input Simulation
[yStep, tStep] = step(linSys, timeVec);
yStep = moveOutput(yStep, shift);
%plotStates(yStep, tStep, 'Uncontrolled Unit Step Simulation');

%%  Unit Impulse Input Simulation
[yImpulse, tImpulse] = impulse(linSys, timeVec);
yImpulse = moveOutput(yImpulse, shift);
%plotStates(yImpulse, tImpulse, 'Uncontrolled Unit Impulse Simulation');

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
%plotStates(yStepPole, tStepPole, 'Pole Placement Controller Unit Step Simulation');

%  Unit Impulse Input Simulation
[yImpulsePole, tImpulsePole] = impulse(closeLoopSysPolePlace, timeVec);
yImpulsePole = moveOutput(yImpulsePole, shift);
%plotStates(yImpulsePole, tImpulsePole, 'Pole Placement Controller Unit Impulse Simulation');

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
periodicDisturbanceAmp = 2;
periodicDisturbanceFreq = 10;

% Define impulse input disturbance amplitude
impulseAmp = 0;

% Run Simulink model
out = sim('stateFeedbackIntegralControl.slx');

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

%% LQR Controller

%% LQR Controller Design Plots

%% System Response Analysis
% Non Unit Impulse
nonUnitImp = genImpulseInput(timeVec, nonUnitImpulseAmp, pertTime);
[yImpPole, tImpPole] = lsim(closeLoopSysPolePlace, nonUnitImp, timeVec);
yImpPole = moveOutput(yImpPole, shift);
plotStates(yImpPole, tImpPole, 'Test Big Impulse Plot');

% Non Unit Step
nonUnitStep = genStepInput(timeVec, stepAmp, pertTime);
[yStepPole, tStepPole] = lsim(closeLoopSysPolePlace, nonUnitStep, timeVec);
yStepPole = moveOutput(yStepPole, shift);
plotStates(yStepPole, tStepPole, 'Test Big Step Plot');

% Ramp
ramp = genRampInput(timeVec, pertTime);
[yRampPole, tRampPole] = lsim(closeLoopSysPolePlace, ramp, timeVec);
yRampPole = moveOutput(yRampPole, shift);
plotStates(yRampPole, tRampPole, 'Test Ramp Plot');

% low freq sine
lowFreqSine = genSinusodialInput(timeVec, pertTime, 5, 100);
[ySinePole, tSinePole] = lsim(closeLoopSysPolePlace, lowFreqSine, timeVec);
ySinePole = moveOutput(ySinePole, shift);
plotStates(ySinePole, tSinePole, 'Test Low Freq Sine Plot');

% high freq sine
highFreqSine = genSinusodialInput(timeVec, pertTime, 5, 1000);
[ySinePole2, tSinePole2] = lsim(closeLoopSysPolePlace, highFreqSine, timeVec);
ySinePole2 = moveOutput(ySinePole2, shift);
plotStates(ySinePole2, tSinePole2, 'Test High Freq Sine Plot');

%% System Response Analysis Plots
% TODO: Fill in this section -> make a function that plots both Pole & LQR