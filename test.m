load(filename)
%%
load('Acc_bns.mat');
load('Gyro_bns.mat');

% Resampling control data
time_ctrl = data.Control.ROSTime;
time_imu = data.IMU_CalData.ROSTime;
time_opti = data.Optitrack_Pose.ROSTime;
dw_time = data.DW_data.ROSTime;

ts_C_roll = timeseries(data.Control.Control.roll,time_ctrl);
ts_C_pitch = timeseries(data.Control.Control.pitch,time_ctrl);
ts_C_yaw = timeseries(data.Control.Control.yaw,time_ctrl);
ts_C_thrust = timeseries(data.Control.Control.thrust,time_ctrl);

ts_C_roll = resample(ts_C_roll,time_imu);
ts_C_pitch = resample(ts_C_pitch,time_imu);
ts_C_yaw = resample(ts_C_yaw,time_imu);
ts_C_thrust = resample(ts_C_thrust,time_imu);

ts_C_X = timeseries(data.Optitrack_Pose.Position.X,time_opti);
ts_C_Y = timeseries(data.Optitrack_Pose.Position.Y,time_opti);
ts_C_Z = timeseries(data.Optitrack_Pose.Position.Z,time_opti);

ts_C_X = resample(ts_C_X,time_imu);
ts_C_Y = resample(ts_C_Y,time_imu);
ts_C_Z = resample(ts_C_Z,time_imu);

data_out = data;

data_out.Control.ROSTime = time_imu;
data_out.Control.Control.roll = ts_C_roll.Data(:);
data_out.Control.Control.pitch = ts_C_pitch.Data(:);
data_out.Control.Control.yaw = ts_C_yaw.Data(:);
data_out.Control.Control.thrust = ts_C_thrust.Data(:)*10;

data_out.Optitrack_Pose.Position.X = ts_C_X.Data(:);
data_out.Optitrack_Pose.Position.Y = ts_C_Y.Data(:);
data_out.Optitrack_Pose.Position.Z = ts_C_Z.Data(:);
data_out.Optitrack_Pose.ROSTime = dw_time;



data_out.IMU_RawData.Acc.X = data_out.IMU_RawData.Acc.X*X_acc(1) + X_acc(2);
data_out.IMU_RawData.Acc.Y = data_out.IMU_RawData.Acc.Y*X_acc(3) + X_acc(4);
data_out.IMU_RawData.Acc.Z = data_out.IMU_RawData.Acc.Z*X_acc(5) + X_acc(6);

data_out.IMU_RawData.Gyro.X = data_out.IMU_RawData.Gyro.X*X_gyro(1) + X_gyro(2);
data_out.IMU_RawData.Gyro.Y = data_out.IMU_RawData.Gyro.Y*X_gyro(3) + X_gyro(4);
data_out.IMU_RawData.Gyro.Z = data_out.IMU_RawData.Gyro.Z*X_gyro(5) + X_gyro(6);

data_out.DW_data.DW1.range(:,1) = data_out.DW_data.DW1.range(:,1)/1000;
data_out.DW_data.DW2.range(:,1) = data_out.DW_data.DW2.range(:,1)/1000;
data_out.DW_data.DW3.range(:,1) = data_out.DW_data.DW3.range(:,1)/1000;
data_out.DW_data.DW4.range(:,1) = data_out.DW_data.DW4.range(:,1)/1000;


%%
%st = 0
%ed = 0
time = time_imu(1:end-1) -  time_imu(1);
X = DAQ_EKF.state_vec(1,:);
X_g = data_out.Optitrack_Pose.Position.X(1:end-1)';

eX = X_g - X;
figure
plot(time,X_g)
hold on
plot(time,X)

Y = DAQ_EKF.state_vec(2,:);
Y_g = -data_out.Optitrack_Pose.Position.Y(1:end-1)';
eY = Y_g - Y;
eX = X_g - X;
figure
plot(time,Y_g)
hold on
plot(time,Y)

Z = -DAQ_EKF.state_vec(3,:);
Z_g = -data_out.Optitrack_Pose.Position.Z(1:end-1)';
eZ = Z_g - Z;
eX = X_g - X;
figure
plot(time,Z_g)
hold on
plot(time,Z)

covX = 3*sqrt(DAQ_EKF.pos_cov(1,:));
covY = 3*sqrt(DAQ_EKF.pos_cov(2,:));
covZ = 3*sqrt(DAQ_EKF.pos_cov(3,:));
figure
plot(time,eX)
hold on
plot(time,covX);
plot(time,-covX);

figure
plot(time,eY)
hold on
plot(time,covY);
plot(time,-covY);

figure
plot(time,eX)
hold on
plot(time,covZ);
plot(time,-covZ);
