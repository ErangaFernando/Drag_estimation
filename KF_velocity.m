%% This script will estimate the velocity of the quadrotor using the position from the optitrack
% data should be available
%% State vector
% X     : X position
% Y     : Y position
% Z     : Z position
% Vx 	: X velocity
% Vy 	: Y velocity
% Vz 	: Z velocity

% Load data
if (exist('data') ~= 1)
	filename = 'quadflight20180822_1.mat';
	load(filename)
end
% Define matrices
A = eye(9);
A(1,4) = 1; A(2,5) = 1; A(3,6) = 1;
C = [eye(3) zeros(3,6)];
measurement = zeros(3,1);
% Initializing the state vector
X = data.optitrack_pose.Position.X(1);
Y = data.optitrack_pose.Position.Y(1);
Z = data.optitrack_pose.Position.Z(1);
Vx = 0; Vy = 0; Vz = 0; Ax = 0; Ay = 0; Az = 0;
state = [X Y Z Vx Vy Vz Ax Ay Az]'

% Noise parameters
Q = eye(9)*100; Q(1:6,1:6) = zeros(6);
R = eye(3)*(1e-8);
P_est = eye(9);

init_time = data.optitrack_pose.ROSTime(1);
time = data.optitrack_pose.ROSTime(1:end) - init_time;
dt = time(2:end) - time(1:end-1);

length = size(data.optitrack_pose.ROSTime,1)

% Output vector
V = zeros(length,3);
meas = zeros(length,3);
meas(1,:) = [X Y Z]';
for i = 2:length

	% Prediction
    dt_i = dt(i-1);
    A(1:6,4:9) = eye(6)*dt_i;
	state_pred = A*state;
	P_pred = A*P_est*A' + Q;
	% Estimating the Kalman gain
	% Defining the noise covariance
	Rdt = R/sqrt(dt_i);
	S = C*P_pred'*C' + Rdt;
	B = C*P_pred;
	K = (S\B)';
	% Measurement
	measurement = [data.optitrack_pose.Position.X(i) data.optitrack_pose.Position.Y(i) data.optitrack_pose.Position.Z(i)]';
	% Correcting the state
	state = state_pred + K*(measurement - C*state_pred);
	% Covariance propagation
	P_est = P_pred - K*C*P_pred;
	V(i-1,:) = state(4:6);
    meas(i,:) = meas(i-1,:) + V(i-1,:)*dt_i; 
end

figure
plot(time,data.optitrack_pose.Position.X)
hold on
plot(time,meas(:,1))
figure
plot(time,data.optitrack_pose.Position.Y)
hold on
plot(time,meas(:,2))
figure
plot(time,data.optitrack_pose.Position.Z)
hold on
plot(time,meas(:,3))

ex = data.optitrack_pose.Position.X - meas(:,1);
ey = data.optitrack_pose.Position.Y - meas(:,2);
ez = data.optitrack_pose.Position.Z - meas(:,3);
figure
plot(time,ex)
figure
plot(time,ey)
figure
plot(time,ez)

rms(ex)
rms(ey)
rms(ez)
