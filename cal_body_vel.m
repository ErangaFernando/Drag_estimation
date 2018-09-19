function [vel_body acc_body] = cal_body_vel(ROSTime, pos, orientation)
%% Inputs
% ROSTime 	: Time
% pos 		: Position in world frame
% orientation : Roll, Pitch, Yaw of the quadrotor inworld frame
%% State vector
% X     : X position
% Y     : Y position
% Z     : Z position
% Vx 	: X velocity
% Vy 	: Y velocity
% Vz 	: Z velocity
% Ax	: X acceleration
% Ay	: y acceleration
% Az	: z acceleration
% Define matrices
A = eye(9);
A(1,4) = 1; A(2,5) = 1; A(3,6) = 1;
C = [eye(3) zeros(3,6)];
measurement = zeros(3,1);
% Initializing the state vector
X = pos(1,1)
Y = pos(1,2);
Z = pos(1,3);
Vx = 0; Vy = 0; Vz = 0; Ax = 0; Ay = 0; Az = 0;
state = [X Y Z Vx Vy Vz Ax Ay Az]';

% Noise parameters
Q = eye(9)*100; Q(1:6,1:6) = zeros(6);
R = eye(3)*(1e-8);
P_est = eye(9);

init_time = ROSTime(1);
time = ROSTime(1:end) - init_time;
dt = time(2:end) - time(1:end-1);

length = size(ROSTime,1)

% Output vector
V = zeros(length,3);
Acc = zeros(length,3);
vel_body = zeros(length,3);
acc_body = zeros(length,3);
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
	measurement = [pos(i,1) pos(i,2) pos(i,3)]';
	% Correcting the state
	state = state_pred + K*(measurement - C*state_pred);
	% Covariance propagation
	P_est = P_pred - K*C*P_pred;
	V(i-1,:) = state(4:6);
	Acc(i-1,:) = state(7:9);
	% Testing estimation
    %meas(i,:) = meas(i-1,:) + V(i-1,:)*dt_i; 

    % Transforming world velocities to the body frame velocities
    phi = orientation(i-1,1); theta = orientation(i-1,2); psi = orientation(i-1,3);
    R_eb = [[ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)];
			[ cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)];
			[         -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta)]];
	vel_body(i-1,:) = V(i-1,:)*R_eb; %(R_eb'*V(i-1,:)')'
	acc_body(i-1,:) = Acc(i-1,:)*R_eb; %(R_eb'*V(i-1,:)')'
end

	%vel_body = quatrotate([orientation(:,4) orientation(:,1:3)], V);

end