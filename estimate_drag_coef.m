function drag_coefs = estimate_drag_coef(filename)

%% This script will calculate the drag coefficient
%filename = 'quadflight20180906_2.mat'
load(filename);
load('Acc_bns.mat');

IMU_time = data.IMU_CalData.ROSTime;
opti_time = data.Optitrack_Pose.ROSTime;

%% Calculate the offset between optitrack and quadrotor
% Starting and end index of stationary segment
st = 1;
ed = 10;
if strcmp(filename,'quadflight20180906_1.mat')
    st = 60;
    ed = 200;
elseif strcmp(filename,'quadflight20180906_2.mat')
    st = 30;
    ed = 110;
end
angle_offset = estimate_rpy_offset(data,st,ed)

%% Transform the coordinates of the optitrack, North-West-Up to quadrotor coordinates North-East-Down
% Position
pos = [data.Optitrack_Pose.Position.X -data.Optitrack_Pose.Position.Y -data.Optitrack_Pose.Position.Z];

% Orientation
opti_orien = [data.Optitrack_Pose.Orientation.X data.Optitrack_Pose.Orientation.Y data.Optitrack_Pose.Orientation.Z data.Optitrack_Pose.Orientation.W];
%%%%%%%%%% This transformation does not work
%ql = quatM_L([1 0 0 0]);
%orientation = (ql*opti_orien')';
%rpy_opti = quat2eul([orientation(:,4) orientation(:,1:3)],'ZYX');
%rpy_opti = [(rpy_opti(:,3) + angle_offset(1)) (-rpy_opti(:,2) + angle_offset(2)) (-rpy_opti(:,1) + angle_offset(3))];
%%%%%%%%%%%%%%%
rpy_imu = [data.IMU_CalData.Angle.Roll data.IMU_CalData.Angle.Pitch data.IMU_CalData.Angle.Yaw]*pi/180;
rpy_opti = quat2eul([opti_orien(:,4) opti_orien(:,1:3)],'ZYX');
rpy_opti = [(rpy_opti(:,3)+ angle_offset(1)) (-rpy_opti(:,2)+ angle_offset(2)) (-rpy_opti(:,1)+ angle_offset(3))];



[vel_b b_acc] = cal_body_vel(data.Optitrack_Pose.ROSTime,pos,rpy_opti);

%% Calculating the bias
% Resampling the optitrac data to match the IMU data
ts_vel_b_x = timeseries(vel_b(:,1),opti_time);
ts_vel_b_y = timeseries(vel_b(:,2),opti_time);
ts_vel_b_z = timeseries(vel_b(:,3),opti_time);

ts_vel_b_x = resample(ts_vel_b_x,IMU_time);
ts_vel_b_y = resample(ts_vel_b_y,IMU_time);
ts_vel_b_z = resample(ts_vel_b_z,IMU_time);

if(isnan(ts_vel_b_x.Data(1)))
    ts_vel_b_x.Data(1) = ts_vel_b_x.Data(2);
end
if(isnan(ts_vel_b_y.Data(1)))
    ts_vel_b_y.Data(1) = ts_vel_b_y.Data(2);
end
if(isnan(ts_vel_b_z.Data(1)))
    ts_vel_b_z.Data(1) = ts_vel_b_z.Data(2);
end
if(isnan(ts_vel_b_x.Data(end)))
    ts_vel_b_x.Data(end) = ts_vel_b_x.Data(end-1);
end
if(isnan(ts_vel_b_x.Data(end)))
    ts_vel_b_x.Data(end) = ts_vel_b_x.Data(end-1);
end
if(isnan(ts_vel_b_x.Data(end)))
    ts_vel_b_x.Data(end) = ts_vel_b_x.Data(end-1);
end


new_vel_b = [ts_vel_b_x.Data ts_vel_b_y.Data ts_vel_b_z.Data];
imu_acc = [data.IMU_CalData.Acc_calb.X data.IMU_CalData.Acc_calb.Y data.IMU_CalData.Acc_calb.Z];

new_vel_cal = [];
new_vel_test = [];
imu_acc_cal = [];
imu_acc_test = [];
rpy_opti_cal = [];
rpy_opti_test = [];
new_time_cal = [];
new_time_test = [];
for i = 1:size(imu_acc)
    if(mod(i,2))
        new_vel_cal = [new_vel_cal; new_vel_b(i,:)];
        imu_acc_cal = [imu_acc_cal; imu_acc(i,:)];
        rpy_opti_cal = [rpy_opti_cal; rpy_opti(i,:)];
        new_time_cal = [new_time_cal; IMU_time(i)];
    else
        new_vel_test = [new_vel_test; new_vel_b(i,:)];
        imu_acc_test = [imu_acc_test; imu_acc(i,:)];
        rpy_opti_test = [rpy_opti_test; rpy_opti(i,:)];
        new_time_test = [new_time_test; IMU_time(i)];
    end
end

X0 = [10.1,100.1];
options = optimoptions('lsqnonlin','Display','iter','UseParallel',true,...
    'Algorithm','levenberg-marquardt');

[X,resnorm,residual,exitflag,output] = lsqnonlin(@(X)cal_err(X,imu_acc_test,new_vel_test,rpy_opti_test),X0);
X;

Xd = diag(X);
err = zeros(size(rpy_opti_test	,1),2);
cal_acc = zeros(size(rpy_opti_test	,1),2);
for i = 1:size(rpy_opti_test,1)
    phi = rpy_opti_test(i,1);
    theta = rpy_opti_test(i,2);
    psi = rpy_opti_test(i,3);
    R_eb = [[ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)];
        [ cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)];
        [         -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta)]];
    g_acc = R_eb'*[0,0,1]';
    cal_acc(i,:) = -new_vel_test(i,1:2)*Xd + g_acc(1:2)';
    err(i,:) = imu_acc_test(i,1:2) - cal_acc(i,:);
end

figure
plot(vel_b(:,1))
hold on
plot(vel_b(:,2))
plot(vel_b(:,3))


figure
plot(IMU_time,data.IMU_CalData.Acc_calb.X);
hold on
plot(new_time_test,cal_acc(:,1));

figure
plot(IMU_time, data.IMU_CalData.Acc_calb.Y);
hold on
plot(new_time_test,cal_acc(:,2));
%{
figure
plot(IMU_time, data.IMU_CalData.Acc_calb.Z);
hold on
plot(opti_time, b_acc(:,3)/90.81);
%}



figure
plot(err(:,1));
figure
plot(err(:,2));
mean(err)

drag_coefs = X;
end

function F = cal_err(X,imu_acc,vel_b,rpy)
err = zeros(size(rpy,1),2);
for i = 1:size(rpy,1)
    phi = rpy(i,1);
    theta = rpy(i,2);
    psi = rpy(i,3);
    R_eb = [[ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)];
        [ cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)];
        [         -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta)]];
    g_acc = R_eb'*[0,0,1]';
    err(i,:) = imu_acc(i,1:2) + vel_b(i,1:2)*diag(X) - g_acc(1:2)';
end

F = reshape(err,1,numel(err));
end