close all; clear; clc;

%% Pole Placement Design for Second-Order System

A = [-1, -2; 1, 0];
B = [2; 0];
C = [0, 1];
D = 0;

sys = ss(A, B, C, D);

figure(1)
hold on;
step(sys)

Pol = pole(sys);

p = [-1, -2];

K = place(A, B, p);
Acl = A-B*K;
syscl = ss(Acl, B, C, D);
Pcl = pole(syscl);

figure(1)
step(syscl)

p = [-2, -3];
K2 = place(A,B,p);
syscl2 = ss(A-B*K2,B,C,D);
figure(1);
step(syscl2);

stepinfo(syscl)
stepinfo(syscl2)

%% Pole Placement Observer Design
A = [-1, -0.75; 1, 0];
B = [1; 0];
C = [1, 1];
D = 0;

Plant = ss(A, B, C, D);

N = 250;
t = linspace(0,25,N);
u = [ones(N/2,1); zeros(N/2,1)];
x0 = [1;2];
[y,t,x] = lsim(Plant,u,t,x0);

figure(2)
plot(t,y);
title('Output');

L = place(A',C',[-2,-3])';

At = A-L*C;
Bt = [B,L];
Ct = [C;eye(2)];
sysObserver = ss(At,Bt,Ct,0);

[observerOutput,t] = lsim(sysObserver,[u,y],t);
yHat = observerOutput(:,1);
xHat = observerOutput(:,[2 3]);

figure;
plot(t,x);
hold on;
plot(t,xHat,'--');
legend('x_1','x_2','xHat_1','xHat_2')
title('Comparison - Actual vs. Estimated');