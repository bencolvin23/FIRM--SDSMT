% Ben Colvin
% FIRM Research Project
% Nonlinear Dynamic Inversion Control of a AUV
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code will act as the startup code to initialoze parameters in the
% workspace
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The AUV is modeled after the Monterey Bay Aquarium Research Institute (MBARI)
% global sets the parameters to be available across all functions and
% models
%
global L
L = 5.554;   % length [meters]
global dia
dia = 0.533;   % Diameter [meters]
global m
m = 1093.1;  % mass [kg]
global Ix
Ix = 36.677; % moment of inertia about Xb [kg*m^2]
global Iy
Iy = 2154.3; % Moment of inertia baout Yb [kg*m^2]
global Iz
Iz = 2154.3; % Moment of inertia about Zb [kg*m^2]
global Tp
Tp = 52;     % Thrust [N]
global Mwd;
Mwd = -40.161; % added mass [kg m]
global Mrp
Mrp = 107.486021;  % [kg m^2]
global Mq
Mq = 979.399749; % [kg m]
global Mqd
Mqd = -2014.611163; % added mass [kg m^2]
global Mw
Mw = 355.915798;   %  [kg]
global MwwQ
MwwQ = -486.816877; %  Quadratic Drag coeffecient[kg]
global MqqQ
MqqQ = -2850.198251; % Quadratic Drag coeffecient[kg m^2]
global Nrd
Nrd = -2019.194419; % added mass [kg m^2]
global Nvd
Nvd = -71.457236; % added mass [kg m]
global Npq
Npq = -107.486021; % [kg m^2]
global Nr
Nr = -2103.861310; % [kg m]
global Nv
Nv = -279.411748; % [kg]
global NrrQ
NrrQ = -2850.198251; % Quadratic Drag [kg m^2]
global NvvQ
NvvQ = 486.816877;   % Quadratic Drag [kg]
global xg
xg = 0.120877; % center of gravity [m]
global xb
xb = 0.120877; % center of bouyancy [m]
global yg
yg = 0; % center of gravity [m]
global yb
yb = 0; % center of bouyancy [m]
global zg
zg = 0.0065;    % center of gravity [m]
global zb
zb = 0; % center of bouancy [m]
global W
W = m*9.81; % weight of UUV [N]
global B
B = W; % Bouyancy force [N]

% Max deflection of delta_theta and delta_psi = 15 deg