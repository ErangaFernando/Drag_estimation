%% This will linearize the system for the EKF

clc

syms phi theta psi real     % Euler angles, Roll, Pitch, Yaw
syms Xe Ye Ze real          % 3D coordinates of the quadrotor
syms g_x g_y g_z real       % Gyro readings
syms a_x a_y a_z real       % Accelerometer readings
syms b_gx b_gy b_gz real    % Gyro bias
syms b_ax b_ay b_az real    % Accelerometer bias
syms w_gx w_gy w_gz real    % Gyro noise
syms w_ax w_ay w_az real    % Acceleration noise
syms w_bgx w_bgy w_bgz real % Gyro bias noise
syms w_bax w_bay w_baz real % Gyro bias noise
syms ge real                % Gravitational acceleraton
syms tau real

%% Vectors and matrices ---------------------------------------------------
gyro_rates = [g_x g_y g_z]';
gyro_bias = [b_gx b_gy b_gz]';
gyro_noise = [w_gx w_gy w_gz]';
gyro_bias_noise = [w_bgx w_bgy w_bgz]';
acc_meas = [a_x a_y a_z]';
acc_noise = [w_ax w_ay w_az]';
acc_bias = [b_ax b_ay b_az]';
acc_bias_noise = [w_bax w_bay w_baz]';
e1 = [1 0 0]';
e2 = [0 1 0]';
e3 = [0 0 1]';

% Matrix for converting angular rates to Euler angle rates %% Correct
Omega = [1      tan(theta)*sin(phi)     tan(theta)*cos(phi) ; ...
         0      cos(phi)                -sin(phi)           ; ...
         0      sin(phi)/cos(theta)     cos(phi)/cos(theta) ];

% Rotation matrix %% Correct
R_eb=rotsym('z',psi)*rotsym('y',theta)*rotsym('x',phi);

%% System 1
% Orientation 
% Dynamic equation
f = [	Omega*(gyro_rates - gyro_bias);
		-(1/tau)*gyro_bias;
		zeros(3,1)];
% Dynamic equation qith noise
f_noise = [	Omega*(gyro_bias - gyro_bias + gyro_noise);
			-(1/tau)*gyro_bias + gyro_bias_noise;
			acc_bias_noise];
% State vector
X = [phi theta psi b_gx b_gy b_gz b_ax b_ay b_az]';
% Input noise vector
W = [w_gx w_gy w_gz w_bgx w_bgy w_bgz]';

% Measurement model
% Accelerometer measurement
h = [R_eb'*ge*e3 + acc_bias] + acc_noise;