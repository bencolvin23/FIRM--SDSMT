function [uvw_dot, pqr_dot,Omega_Pow,FT] = test(uvw,pqr,ptp,Real,Imag,z)
%{
This code simulates the body dyanics of a quadcopter
INPUTS:
    uvw - 3x1 vector of body fixed translation velocities u, v, w
    pqr - 3x1 vector of body fixed rotation velocities p, q, r
    ptp - 3x1 vector of world fixed orientations phi, theta, psi
    Omega_Pow - 4x1 vector of  propeller angular velocities

OUTPUTS:
    uvw_dot - 3x1 vector of derivatives of body velocities u, v, w
    pqr_dot - 3x1 vector of derivatives of body rotation velocities, p, q, r
%}

% CONSTANTS
m = 2.3;              % Mass [kg]
g = 9.81;             % Gravity [m/s^2]
L = 0.7;              % Lever arm length [m]
Ixx = 8.04*10^(-3);   % Body inertia of x axis [kg*m^2]
Iyy = 8.46*10^(-3);   % Body inertia of y axis [kg*m^2]
Izz = 14.68*10^(-3);  % Body inertia of y axis [kg*m^2]
k_f = 0.65016*10^(-3);    % Force Coeffecient [N/(rad/s)^2]
k_t = 0.82218*10^(-5);     % Torque Coeffecient [N/(rad/s)^2]
k1 = 20;              % Motor model coeffecient
k2 = 0.01;            % Motor model coeffecient
k3 = 3.5;             % Motor model coeffecient
z_i = 0.003;          % Elastic deformation [m]
k_i = 6000;           % Elastic Coeffecient of gear [N/m]

% INITIALIZATION

% Gets values from matrix
u = uvw(1);
v = uvw(2);
w = uvw(3);

p = pqr(1);
q = pqr(2);
r = pqr(3);

phi = ptp(1);
theta = ptp(2);
psi = ptp(3);

% Deformation of landing gear
% If on the ground/landing
if (z_i - z)> 0
    delta = 1;
% If in the air
else
    delta = 0;
end

Omega_Pow = zeros(4,1);
Omega_Pow = [Real(1)+Imag(1)*i; Real(2)+Imag(2)*i; Real(3)+Imag(3)*i; Real(4)+Imag(4)*i];


% MAIN CODE

% Uses angular speeds of propellers to map forces
FT = [-L*k_f L*k_f L*k_f -L*k_f;...
    -L*k_f L*k_f -L*k_f L*k_f;...
    k_t k_t -k_t -k_t;...
    k_f k_f k_f k_f]*(Omega_Pow.^2);

% Sets forces
Tp = FT(1);
Tq = FT(2);
Tr = FT(3);
Fz = FT(4);

% Body Fixed Translational Dynamics
uvw_dot = [-r*v+q*w; r*u-p*w; -q*u+p*v]+[0; 0; Fz/m]+[g*sin(theta); -g*cos(theta)*sin(psi); -g*cos(theta)*sin(psi)]+[0; 0; k_i*(z_i-z)*delta];

% Body Fixed Rotational Dynamics
pqr_dot = -[q*r*(Izz-Iyy)/Ixx; p*r*(Ixx-Izz)/Iyy; p*q*(Iyy-Ixx)/Izz]+[Tp/Ixx; Tq/Iyy; Tr/Izz];


