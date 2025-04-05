% Main script for ME 577 Group Final Project
close all; clear; clc;

%% User Inputs
% Initial Conditions
ICs = [0; 0; deg2rad(180); 0];

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
timeVec = 0:1E-3:2;

% System Response Parameters
stepAmp = 1;
stepTime = 1;
impulseAmp = 1;

%% System Definition
m = 0.35;
M = 2.2;
L = 1.3;
b = 0.25;
g = 9.8;
I = (1./3).*m.*L.*L;

%% NonLinear EOM Simulation - Newtonian Dynamics Model

%% Linear EOM Simulation - Newtonian Dynamics Model
denom = (I.*(M+m)) + (M.*m.*(L.^2));
A11 = 1;
A22 = (-1.*(I+(m.*(L.^2))).*b)./denom;
A23 = ((m.^2).*g.*(L.^2))./denom;
A34 = 1;
A42 = -1.*(m.*L.*b)./denom;
A43 = (m.*g.*L.*(M+m))./denom;
A = zeros(4);
A(1,2) = A11;
A(2,2) = A22;
A(2,3) = A23;
A(3,4) = A34;
A(4,2) = A42;
A(4,3) = A43;

% B Matrix
B21 = (I+(m.*(L.^2)))./denom;
B41 = (m.*L)/denom;
B = zeros(4,1);
B(2,1) = B21;
B(4,1) = B41;

% C Matrix
%C = zeros(2, 4);
%C(1,1) = 1;
%C(2,3) = 1;
C = eye(4);

% D Matrix
%D = zeros(2,1);
D = zeros(4, 1);

% Create a Matlab State Space Model
linSys = ss(A,B,C,D);

%% Break Linear Assumption
% Show IC's vs Time where linear assumptions break

%% Stability -> HW 4 type of analysis
[eigVectors, eigValues] = eig(A);
isLinSysStable = isstable(linSys);

%% Controlability -> HW 5 type of analysis
P = [B, A*B, A*A*B, A*A*A*B];
rankOfP = rank(P);

%% Observability -> HW 5 type of analysis
Q = [C;...
    C*A;...
    C*A*A;...
    C*A*A*A];
rankOfQ = rank(Q);

%% Initial Condition Response
% [yIc,tIc] = initial(linSys, IC);
% figure()
% plot(tIc, yIc);
% grid on
% xlabel('Time [sec]')
% ylabel('Initial Condition Response')
% 
% %% Step Response
% [yStep,tStep] = step(linSys);
% %stepInfo = stepinfo(linSys); -> only works for stable system needs to
% %reach steady state
% figure()
% plot(tStep, yStep);
% grid on
% xlabel('Time [sec]')
% ylabel('Step Response')
% 
% %% Impulse Response
% [yImpulse, tImpulse] = impulse(linSys);
% figure()
% plot(tImpulse, yImpulse);
% grid on
% xlabel('Time [sec]')
% ylabel('Impulse Response')
% 
% %% General Response
% tGeneralResponse = 0:1E-3:10;
% omega = 2*pi;
% uGeneralResponse = sin(omega*tGeneralResponse);
% yGeneralResponse = lsim(linSys, uGeneralResponse, tGeneralResponse);
% figure()
% plot(tGeneralResponse, yGeneralResponse);
% grid on
% xlabel('Time [sec]')
% ylabel('General Response')

%% Pole Placement
zeta = (-1*log(desPO./100))/((pi.^2) + (desPO./100));
wn = (-1*log(inputPercent./100))./(tSettle.*zeta);
linSysPole = pole(linSys);
eqn = [1, 2.1, 3.4, 2.7.*wn, wn.^2];
p = roots(eqn);
stableP = p(real(p) == min(real(p)));
p = [stableP(1); stableP(2);...
    poleScalarOne*min(real(stableP));...
    poleScalarTwo*min(real(stableP))];
[K, prec] = place(A,B,p);
closeLoopA = A - (B*K);
closeLoopSysPolePlace = ss(closeLoopA, B, C, D);

%% LQR Controller

%% Controller Design Plots
% Controller Design Plots
figure();
hold on
step(closeLoopSysPolePlace)
title('Unit Step Input')
grid on

figure();
hold on
impulse(closeLoopSysPolePlace)
title('Unit Impulse Input')
grid on

%% System Response
% This analysis shows the comparison of the Pole Placement Method and LQR
% Method for the Close-Loop Controller with the Disturbance Plot

% Step Response
stepInput = genStepInput(timeVec, stepTime, stepAmp);
[yOpenStep, tOpenStep] = lsim(linSys, stepInput, timeVec, ICs);
[yClosePoleStep, tClosePoleStep] = lsim(closeLoopSysPolePlace, stepInput,...
    timeVec, ICs);

figure()
scatter(timeVec, stepInput)
grid on
xlabel('Time [sec]')
ylabel('Response')
title('Step Response')

figure();
title('Step Response')
subplot(4, 1, 1)
hold on
plot(tOpenStep, yOpenStep(:, 1), 'k')
plot(tClosePoleStep, yClosePoleStep(:, 1), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Position [m]')

subplot(4, 1, 2)
hold on
plot(tOpenStep, yOpenStep(:, 2), 'k')
plot(tClosePoleStep, yClosePoleStep(:, 2), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Velocity [m/s]')

subplot(4, 1, 3)
hold on
plot(tOpenStep, yOpenStep(:, 3), 'k')
plot(tClosePoleStep, yClosePoleStep(:, 3), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Angle [deg]')

subplot(4, 1, 4)
hold on
plot(tOpenStep, yOpenStep(:, 3), 'k')
plot(tClosePoleStep, yClosePoleStep(:, 3), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Angle Rate [deg/sec]')

% Impule Response
impulseInput = genImpulseInput(timeVec, 1, impulseAmp);
[yOpenImpulse, tOpenImpulse] = lsim(linSys, impulseInput, timeVec, ICs);
[yClosePoleImpulse, tClosePoleImpulse] = lsim(closeLoopSysPolePlace, impulseInput,...
    timeVec, ICs);

figure()
scatter(timeVec, impulseInput)
grid on
xlabel('Time [sec]')
ylabel('Response')
title('Impulse Response')

figure();
title('Step Response')
subplot(4, 1, 1)
hold on
plot(tOpenImpulse, yOpenImpulse(:, 1), 'k')
plot(tClosePoleImpulse, yClosePoleImpulse(:, 1), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Position [m]')

subplot(4, 1, 2)
hold on
plot(tOpenImpulse, yOpenImpulse(:, 2), 'k')
plot(tClosePoleImpulse, yClosePoleImpulse(:, 2), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Velocity [m/s]')

subplot(4, 1, 3)
hold on
plot(tOpenImpulse, yOpenImpulse(:, 3), 'k')
plot(tClosePoleImpulse, yClosePoleImpulse(:, 3), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Angle [deg]')

subplot(4, 1, 4)
hold on
plot(tOpenImpulse, yOpenImpulse(:, 3), 'k')
plot(tClosePoleImpulse, yClosePoleImpulse(:, 3), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Angle Rate [deg/sec]')

%% Responses to add
% Ramp
% Triangle
% Saw tooth
% low freq sine
% high freq sine
% low + high freq sine

%% Controller Comparison Response
% These plots only show the comparison of the Pole Placement Method and LQR
% Method for the Close-Loop Control

% Step Plot
figure();
title('Step Response')
subplot(4, 1, 1)
hold on
plot(tOpenStep, yOpenStep(:, 1), 'k')
plot(tClosePoleStep, yClosePoleStep(:, 1), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Position [m]')

subplot(4, 1, 2)
hold on
plot(tOpenStep, yOpenStep(:, 2), 'k')
plot(tClosePoleStep, yClosePoleStep(:, 2), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Velocity [m/s]')

subplot(4, 1, 3)
hold on
plot(tOpenStep, yOpenStep(:, 3), 'k')
plot(tClosePoleStep, yClosePoleStep(:, 3), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Angle [deg]')

subplot(4, 1, 4)
hold on
plot(tOpenStep, yOpenStep(:, 3), 'k')
plot(tClosePoleStep, yClosePoleStep(:, 3), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Angle Rate [deg/sec]')

% Impule Plot
figure();
title('Step Response')
subplot(4, 1, 1)
hold on
plot(tClosePoleImpulse, yClosePoleImpulse(:, 1), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Position [m]')

subplot(4, 1, 2)
hold on
plot(tClosePoleImpulse, yClosePoleImpulse(:, 2), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Velocity [m/s]')

subplot(4, 1, 3)
hold on
plot(tClosePoleImpulse, yClosePoleImpulse(:, 3), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Angle [deg]')

subplot(4, 1, 4)
hold on
plot(tClosePoleImpulse, yClosePoleImpulse(:, 3), 'b')
hold off
grid on
xlabel('Time [s]')
ylabel('Angle Rate [deg/sec]')

