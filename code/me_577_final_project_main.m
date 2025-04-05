close all; clear; clc;

%% User Inputs
IC = [0; 0; deg2rad(180); 0];

%% System Definition
m = 0.2;%0.35;
M = 0.5;%2.2;
L = 0.3;%1.3;
b = 0.1;%0.25;
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
C = zeros(2, 4);
C(1,1) = 1;
C(2,3) = 1;
%C = eye(4);

% D Matrix
D = zeros(2,1);

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

% %% Initial Condition Response
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
desPO = 10;
tSettle = 1;
inputPercent = 2;
zeta = (-1*log(desPO./100))/((pi.^2) + (desPO./100));
%zeta = 0.591328;
wn = (-1*log(inputPercent./100))./(tSettle.*zeta);
% Source: https://www.mathworks.com/help/control/ref/place.html
linSysPole = pole(linSys);
eqn = [1, 2.1, 3.4, 2.7.*wn, wn.^2];
p = roots(eqn);
stableP = p(real(p) == min(real(p)));
p = [stableP(1); stableP(2); 5*min(real(stableP)); 10*min(real(stableP))];
[K, prec] = place(A,B,p);
closeLoopA = A - (B*K);
closeLoopSys = ss(closeLoopA, B, C, D);

figure(1);
hold on
step(closeLoopSys)
grid on
title('Unit Step Input')
stepinfo(closeLoopSys)

figure(2);
hold on
impulse(closeLoopSys)
title('Unit Impulse Input')
grid on


%% PID Controller
%C = pidtune(sys,'PID');

%% LQR Controller

%% System Response

% y = lsim(sys,u,t,IC)
% https://www.mathworks.com/help/control/ref/dynamicsystem.lsim.html