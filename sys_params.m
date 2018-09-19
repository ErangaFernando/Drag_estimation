global anchor_pos 
global tau Ts sqrdt ge m
global sigma_U4 sigma_prop sigma_gx sigma_gy sigma_gz sigma_bgx sigma_bgy sigma_bgz sigma_mx sigma_my sigma_mz sigma_mO
global sigma_ax sigma_ay sigma_az sigma_zh sigma_r sigma_bax sigma_bay sigma_baz
global I invI dt e3
global ref_height
% Aerodynamic coefficients
global b d l k_pp k_pa k_prop
%% Low noise levels
%{
%Time constant of bias
tau = 10;
% Sampling time
Ts = 1/200;
sqrdt = sqrt(0.0823);
% Gravitational acceleration
ge = 9.81;
% Mass of the quadrotor
m = 0.695 - 0.0*0.56; % New model

anchor_pos = [-1.592  -1.480    0.104;
       	   2.130   -0.8283    -0.0568;
       	   1.3794  1.293    -0.053];
% Noise parameters
sigma_U4 = 0.05;
sigma_prop = 4.8;
sigma_gx = 1.6968e-2/sqrdt;
sigma_gy = 1.6968e-2/sqrdt;
sigma_bgx = 1.9393e-3/sqrdt;
sigma_bgy = 1.9393e-3/sqrdt;
sigma_bgz = 1.9393e-3/sqrdt;
sigma_mx = 0.8;
sigma_my = 0.8;
sigma_mz = 0.8;

% Measurement noise
sigma_ax = 2.00e-3/sqrdt;
sigma_ay = 2.00e-3/sqrdt;
sigma_az = 2.00e-3/sqrdt;

sigma_zh = 0.02; % m^2
sigma_r = 0.055; % m^2

%}

%% High noise levels
%Time constant of bias
tau = 0.1;
% Sampling time
Ts = 1/200;
sqrdt = sqrt(0.0823);
% Gravitational acceleration
ge = 9.81;
% Mass of the quadrotor
m = 0.695 - 0.0*0.56; % New model
% Height reference
ref_height = -2;

anchor_pos = [-1.592  -1.480    -0.104;
       	   2.130   -0.8283    -0.0568;
       	   1.3794  1.293    -0.053];
% Noise parameters
sigma_U4 = 7;
sigma_prop = 0.5;
sigma_gx = 1.6968e-2/sqrdt;
sigma_gy = 1.6968e-2/sqrdt;
sigma_gz = 1.6968e-2/sqrdt;
sigma_bgx = 1.9393e-4/sqrdt;
sigma_bgy = 1.9393e-4/sqrdt;
sigma_bgz = 1.9393e-4/sqrdt;
sigma_bax = 1.9393e-4/sqrdt;
sigma_bay = 1.9393e-4/sqrdt;
sigma_baz = 1.9393e-4/sqrdt;
sigma_mx = 0.8;
sigma_my = 0.8;
sigma_mz = 0.8;
sigma_mO = 100;

% Measurement noise
sigma_ax = 0.4%12.00e-3/sqrdt;
sigma_ay = 0.4%12.00e-3/sqrdt;
sigma_az = 1.7%12.00e-3/sqrdt;

sigma_zh = 0.02; % m^2
sigma_r = 0.1; % m^2

%% Physical parameters
I = diag([0.00367556974 0.00365031574 0.00703095144]);
invI = inv(I);
dt = Ts;
e3 = [0 0 1]';
b = 6.41e-6;
d = 9.31e-7;
l = 0.34;
k_pp = 0.00011;
k_pa = 0.00023;
k_prop = b;