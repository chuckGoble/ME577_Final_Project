close all; clear; clc;

massPen = 0.35;
massCart = 2.2;
lenRod = 1.3;
cartFriction = 0.25;
grav = 9.8;
u = 0;

state_0 = [0;0;deg2rad(0);0];
[t, y] = ode45(@(t, state) cartOnPenEoM(t, state, massPen, massCart, lenRod,...
    cartFriction, grav, u), [0, 10], state_0);

figure()
plot(t, y)
grid on
legend('Pos', 'Vel', 'Angle', 'Angle Rate', 'location', 'BestOutside');


%% Helper Functions
function [dState] = cartOnPenEoM(t, state, massPen, massCart, lenRod,...
    cartFriction, grav, u)
    % massPen = params.massPen;
    % massCart = params.massCart;
    % lenRod = params.lenRod;
    % cartFriction = params.cartFriction;
    % grav = params.grav;
    % u = params.u;

    % Helper Var
    theta = state(3);

    % Differential Equations of Motion
    dState = NaN(size(state));
    dState(1) = state(2);
    one = -1.*(massPen.^2).*(lenRod.^2).*grav.*cos(theta).*sin(theta);
    two = (massPen.*(lenRod.^2)).*((massPen.*lenRod.*(state(4).^2))-(cartFriction.*state(2)));
    three = massPen.*(lenRod.^2).*u;
    num = one + two + three;
    temp = cos(theta).^2;
    temp = 1 - temp;
    temp = massPen.*temp;
    temp = massCart + temp;
    denom = massPen.*(lenRod.^2).*temp;
    dState(2) = num./denom;
    dState(3) = state(3);
    one = (massPen+massCart).*(massPen.*grav.*lenRod.*sin(theta));
    two = (massPen.*lenRod.*cos(theta)).*...
        (massPen.*lenRod.*(state(4).^2).*sin(theta) - (cartFriction.*state(3)));
    three = massPen.*lenRod.*cos(theta).*u;
    num = one + two +  three;
    dState(4) = num./denom;
end