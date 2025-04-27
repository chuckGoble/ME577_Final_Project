close all; clear; clc;

% Constants
grav = -9.8;

% System Definition
massPen = 0.35;
massCart = 2.2;
lenRod = 1.3;
cartFriction = 0.25;

% Input function
u = 0;

% Define Sim Time
tSpan = 0:1E-1:600;

% Use Linear Dynamics Model
% 0 for nonlinear & 1 for linear
isLinear = 0;

% Initial State Vector
state_0 = [0; 0; deg2rad(180); deg2rad(0)];

% Solve Differential Equations of Motion
[t, y] = ode45(@(t, state) invPenCartEoM(t, state, massPen, massCart, lenRod,...
    cartFriction, grav, @zeroInput, isLinear), tSpan, state_0);

%% Post Processing
% Carpet Plot
figure(1)
plot(t, y)
grid on
legend('Pos', 'Vel', 'Angle', 'Angle Rate', 'location', 'BestOutside');

% "movie" plot
figure(2)
for k=1:length(t)
    drawpend(y(k,:), massPen, massCart, lenRod);
end


%% Helper Functions
function [dState] = invPenCartEoM(t, state, massPen, massCart, lenRod,...
    cartFriction, grav, u, linFlag)
% This function is the equations of motion for an inverted pendulum on a
% cart. The cart is subject to a set of initial conditions, an input forcing
% function and friction that opposes the cart's motion.
%
% INPUTS
% ------
% t : double
%   Simulation Time in sec
%
% state : 4x1 array of doubles
%   State vector [x, xdot, theta, thetadot]
%
% massPen : double
%   Mass of the end of the pendulum
%
% massCart : double
%   Mass of the Cart
%
% lenRod : double
%   length of the pendulum's rod
%
% cartFriction : double
%   friction of the cart
%
% grav : double
%   gravity
%
% u : double
%   input forcing function
% linFlag : bool
%   Flag to switch between linear and non linear model
%
% RETURNS
% -------
% dState : 4x1 array of double
%   Time derivative of state vector

    % Evaluation the input function u(t)
    u = feval(u, t);

    % Helper Var
    if linFlag
        cosTheta = 1;
        sinTheta = 0;
    else
        cosTheta = cos(state(3));
        sinTheta = sin(state(3));
    end

    % Compute Common Denominator
    temp = cosTheta;
    temp = temp.^2;
    temp = 1 - temp;
    temp = massPen.*temp;
    temp = massCart + temp;
    denom = massPen.*(lenRod.^2).*temp;

    % Differential Equations of Motion
    dState = NaN(size(state));
    
    % x dot - linear velocity
    dState(1) = state(2);
    
    % x double dot - linear acceleration
    one = -1.*(massPen.^2).*(lenRod.^2).*grav.*cosTheta.*sinTheta;
    two = massPen.*lenRod.*(state(4).^2).*sinTheta;
    two = two - (cartFriction.*state(2));
    two = (massPen.*(lenRod.^2)).*two;
    three = massPen.*(lenRod.^2).*u;
    num = one + two + three;
    dState(2) = num./denom;
    
    % theta dot - angular velocity
    dState(3) = state(4);
    
    % theta double dot - angular acceleration
    one = (massPen+massCart).*(massPen.*grav.*lenRod.*sinTheta);
    two = massPen.*lenRod.*(state(4).^2).*sinTheta;
    two = two - (cartFriction.*state(2));
    two = (massPen.*lenRod.*cosTheta).*two;
    three = massPen.*lenRod.*cosTheta.*u;
    num = one - two +  three;
    dState(4) = num./denom;
end