 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A3S EXAM SCRIPT AY 2025/2026
% Author:  Davide Invernizzi (davide.invernizzi@polimi.it)
% v09/10/2025      
% This file contains data for the exam of the A3S course ay 2025/2026.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;
clc;

%% Nominal Parameters
% Parámetros varios
degToRad = pi/180;
radTodeg = 180/pi;
par.g = 9.81;
par.e_3 = [0 0 1]';
% Condiciones iniciales cinematica
q_0 = eul2quat([0, 0, 0 ]*degToRad,'XYZ')'; %  attitude - quaternion
omegab_0 = [0 0 0]'; %  rad/s  angular velocity (body components)
p_0 = [-24 8 2.5]'; % m position
vb_0 = [0 0 0]'; % m/s linear velocity
R_0 = quat2rotm(q_0');
v_0 = R_0*vb_0;

% UAV inertial and dynamic parameters
% Nominal values for the parameters
UAV.Nr = 6; %number of rotors
UAV.m0 = 1.2; %[kg] UAV mass
UAV.m_p = 0; % Mass of package
UAV.m = UAV.m0 + UAV.m_p; % Total mass
UAV.rc = 0.05; %Distance of package in z direction
UAV.r_bg = [0 0.001 -UAV.m_p*UAV.rc/UAV.m]; %[m] center of mass location in the body frame
J_pkg = UAV.m_p * (UAV.rc^2); % J of package w/r centre of drone
UAV.J = diag([0.01 + J_pkg, 0.01 + J_pkg, 0.018]); % Total J
UAV.S = UAV.m*crossmat(UAV.r_bg); % [kg m] static moment
UAV.M = [UAV.m*eye(3) UAV.S'; % generalized mass matrix
         UAV.S UAV.J];
UAV.Minv = inv(UAV.M); % inverse of mass matrix

% Linear Aerodynamics
UAV.D_tauomega = diag([3.6e-4 3.6e-4 1.20e-2]); % angular velocity damping
UAV.D_fv =  diag([0.26 0.26 0]); % linear velocity damping
UAV.D = [UAV.D_fv zeros(3,3); zeros(3,3) UAV.D_tauomega];

% Propellers 
UAV.b = 0.215; %[m] arm length (ell in the slides)
n = [0; 0; 1];
UAV.Omega_max = 10300*2*pi/60; %[rpm] max spinning rate
UAV.Omega_min = 1260*2*pi/60; %[rpm] min spinning rate
UAV.Omega_0 = 733;
UAV.k_m = 1/0.05; % [s-1] Inverse of the time constant of the propeller motors
k_f = 3.65e-6;                % [N/rad^2/s^2] Thrust characteristic coeff
sigma = 0.09;      % [m] Torque-to-thrust ratio
Lambda_matrix = diag([1, 1, 1, 1,1, 1]); % Propellers efectivenesss

% Allocation matrix
F_matrix = zeros(6, 6);
for i=1:6
    UAV.P_Prop(i, :) = UAV.b*[cos(60*degToRad*(i-1)), sin(60*degToRad*(i-1)), 0];
    UAV.P_Sign(i) = (-1)^(i-1);
    F_matrix(:, i) = [n; (crossmat(UAV.P_Prop(i, :)) - UAV.P_Sign(i)*sigma*eye(3))*n]; 
end
Finv = pinv(F_matrix);

%% Uncertain parameters
% Mass parameters redefinition
m_p_bar = 0.3;
m_p_u = ureal('m_p', m_p_bar, 'Range', [0, 1]);
m_p = 0.5;

r_c_bar = 0.05;
r_c_u = ureal('r_c_u', r_c_bar, 'Range', [0.04, 0.07]);
r_c = usample(r_c_u);

m_t_est = UAV.m0 + m_p;

% Aerodynamic parameters redefinition
D_fv_bar =  diag([0.26 0.26 1e-3]); % linear velocity damping
D_xy_u = ureal('D_xy', D_fv_bar(1,1), 'percent', 50);
D_z_u = ureal('D_z', D_fv_bar(3,3), 'Range', [0, 0.1]);
D_xy = usample(D_xy_u);
D_z = usample(D_z_u);
UAV.D_fv =  diag([D_xy D_xy D_z]); % linear velocity damping
UAV.D = [UAV.D_fv zeros(3,3); zeros(3,3) UAV.D_tauomega];

% Dependent mass parameters update
UAV.m = UAV.m0 + m_p; % Total mass
UAV.rc = r_c;
UAV.r_bg = [0 0 -UAV.m_p*UAV.rc/UAV.m]; %[m] center of mass location in the body frame
J_pkg = UAV.m_p * (UAV.rc^2); % J of package w/r centre of drone
UAV.J = diag([0.01 + J_pkg, 0.01 + J_pkg, 0.018]); % Total J
UAV.S = UAV.m*crossmat(UAV.r_bg); % [kg m] static moment
UAV.M = [UAV.m*eye(3) UAV.S'; % generalized mass matrix
         UAV.S UAV.J];
UAV.Minv = inv(UAV.M); % inverse of mass matrix

LAMBDA_hat_0 = ones(3,1); % Control efectiveness initial estimation
%% Control actitud
%Control  x y
omega_n = 6.25;
xi = 0.8;
ctrl.Krx = UAV.J(1,1)*omega_n^2;
ctrl.Kvx = UAV.J(1,1)*2*xi*omega_n - UAV.D_tauomega(1,1);
ctrl.Kry = ctrl.Krx;
ctrl.Kvy = ctrl.Kvx;
% Control  z
omega_n = 4;
xi = 0.9;
ctrl.Krz = UAV.J(3,3)*omega_n^2;
ctrl.Kvz = UAV.J(3, 3)*2*xi*omega_n - UAV.D_tauomega(3,3);

ctrl.Komega = diag([ctrl.Kvx,ctrl.Kvy,ctrl.Kvz]);
ctrl.Kr = diag([ctrl.Krx,ctrl.Kry,ctrl.Krz]);


%% Control posicion
K_iz=0;
K_dz=3;

% control parameters x y
omega_n_d = 0.6;
xi_d = 1.5;

K_ix = UAV.m*omega_n_d^2;
K_iy = K_ix;
K_dx = UAV.m*2*xi_d*omega_n_d;
K_dy = K_dx;

% Matrices and reference model computation
Kp = diag([K_dx, K_dy, K_dz]);
Ki = diag([K_ix, K_iy, K_iz]);
K_px =0.9;
K_py = K_px;
K_pz = 1;
K_position = diag([K_px, K_py, K_pz]);

Vmax = 5;
A_p_ref = -D_fv_bar;
B_p_ref = 1/m_t_est*diag(LAMBDA_hat_0);
A_ref = [zeros(3,3) eye(3) zeros(3,3); -B_p_ref*Kp*K_position (A_p_ref - B_p_ref*Kp*K_position) B_p_ref*Ki; -K_position -eye(3) zeros(3,3)];
B_ref = [zeros(3,3); B_p_ref*Kp*K_position; K_position];

%% PBMRAC parameters

Theta_0 = [0;0;(m_t_est)*par.g;D_fv_bar(1,1);D_fv_bar(2,2);D_fv_bar(3,3)];
B_p = 1/(m_t_est);
GAMMA_a = diag([0.5, 0.5, 0.5,1, 1, 1, 1, 1, 1, 1]);
L = 100;

% Projection
proj.CenterLambda = 1;
proj.RLambda = 0.5;
proj.eps = 0.1;
proj.CenterCxy = 0;
proj.RCxy = 1.5;
proj.RCZ = 0.1;
proj.CenterDxy = 0.3;
proj.RDxy = 0.1;
proj.CenterDz = 0.022;
proj.RDz = 0.01;
proj.sigma = 0.1;
%% Others
load('trayectoria_referencia.mat');
t_lookup = ts_pos.Time; 


p_lookup = ts_pos.Data; 


p_x_data = p_lookup(:,1);
p_y_data = p_lookup(:,2);
p_z_data = p_lookup(:,3);

wind = [0,0];
%% Simulacion
simout = sim('RB_dyn.slx');
ver_playback(simout)
%% Funciones


function x_cross = crossmat(x)
x_cross=[0 -x(3) x(2);
    x(3) 0 -x(1);
    -x(2) x(1) 0];
end
