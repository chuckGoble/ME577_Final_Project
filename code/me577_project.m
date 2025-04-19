%% Define Constants

clear all
clc

% mass = kg
% length = m
% damping = N-s/m
% acceleration = m/s^2
m = 0.35;
M = 2.2;
L = 1.3;
B = 0.25;
g = 9.81;


%% Define State Space Matrices
A = [0 1 0 0;
    0 -(4*B)/(4*M+m) (3*m*g)/(4*M+m) 0;
    0 0 0 1;
    0 -(6*B)/(L*(4*M+m)) (6*(M+m)*g)/(L*(4*M+m)) 0];

B = [0; 4/(4*M+m); 0; 6/(L*(4*M+m))];

C = [1 0 0 0];

%% Define New Matrices for Integral Error Control

% Define new A and B matrices
Ahat = [A zeros(length(A),1);-C 0];
Bhat = [B;0];

% Check controllabilty of new system. Rank = 5 means that thes system
% is % fully state controllable.
P = [A B;-C 0];
rank(P);

%% Calculate Gain Values

% Define desired eigenvalues for new 5th order system. Eigenvalues were
% defined from values given by
% https://web.mit.edu/16.31/www/Fall06/1631_topic15.pdf that result in a 1
% second settling time
eigDesired = [-6.4480,-4.1104+6.3142i,-4.1104-6.3142i,...
    -5.9268+3.0813i,-5.9268-3.0813i];

% Use place command to calculate gain matrix to acheive desired poles
gainMatrix = place(Ahat,Bhat,eigDesired);

% State feedback gain matrix
K = gainMatrix(1:4);

% Integral error tracking gain value
KI = -gainMatrix(5);

% Redefinie C and D matrices so that all state variables are output for
% plotting purposes in Simulink
C = eye(4);
D = zeros(4,1);

%% Run Simulink Model

% Define desired cart position
cartPosition = 5;

% Define periodic input disturbance amplitude and frequency
periodicDisturbanceAmp = 2;
periodicDisturbanceFreq = 10;

% Define impulse input disturbance amplitude
impulseAmp = 0;

% Run Simulink model
out = sim('stateFeedbackIntegralControl.slx');

figure(1)

subplot(2,1,1)
plot(out.tout,out.command,'--k',out.tout,out.position,'-r')
legend("Commanded Input","Cart Position")

subplot(2,1,2)
plot(out.tout, out.command,'--k',out.tout, out.theta,'r')
legend("Commanded Input","Theta Position")

% Initially created model in Simulink for quick analysis. Will translate to
% Matlab code so that Simulink is not necessary.

%% Old Code for initial state feedback analysis
% % Calculate close loop gain matrix for state feedback controller
% Acl = Ahat-Bhat*gainMatrix;
% 
% % Define state-space model for simulation
% sys = ss(Acl,B,C,D);
% sys.OutputName{1} = 'x (m)';
% sys.OutputName{2} = 'theta (rad)';
% 
% % Define time vector for simulation
% time = 0:.001:50;
% 
% % Define reference input for simulation
% input = 0.32*ones(1,length(time));
% 
% % Define state initial condition vector
% x0 = [0;0;0;0];
% 
% % Run simulation
% y = lsim(sys,input,time,x0);
% 
% figure(1)
% hold on
% grid on
% plot(time,rad2deg(y(:,2)),'-k', LineWidth=1.5)
% ylabel("Pendulum Angle Offset (\circ)")
% xlabel("Time (s)")
% 
% yyaxis right
% plot(time,y(:,1),'-r',LineWidth=1.5)
% ylabel("Cart \Delta Position (m)")
% 
% hold off