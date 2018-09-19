load('quad_stationary_2_edited.mat')
acc_mode = 1
gyro_mode = 1
mag_mode = 1
t = [10,1100;
    1300,1750;
    1850,2000;
    2080,2280;
    2900,3300;
    3400,3800;
    3950,4200 ;
    4350,4600 ;
    4750,5050 ;
    5200,5450;
    7300,7600;
    7700,8100;
    8400,9100;
    9450,10050;
    10250,11050];
time_raw = data.IMU_RawData.Time.sec + data.IMU_RawData.Time.nsec*(1e-9);
time_cal = data.IMU_CalData.Time.sec + data.IMU_CalData.Time.nsec*(1e-9);

if(acc_mode)
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Accelerometer bias
    %% Resampling raw data to the calibrated time
    ts_raw_AccX = timeseries(data.IMU_RawData.Acc.X,time_raw);
    ts_raw_AccY = timeseries(data.IMU_RawData.Acc.Y,time_raw);
    ts_raw_AccZ = timeseries(data.IMU_RawData.Acc.Z,time_raw);
    
    
    
    %ts_cal_AccX = timeseries(data.IMU_CalData.Acc_calb.X,time_cal);
    %ts_cal_AccY = timeseries(data.IMU_CalData.Acc_calb.Y,time_cal);
    %ts_cal_AccZ = timeseries(data.IMU_CalData.Acc_calb.Z,time_cal);
    
    ts_raw_AccX = resample(ts_raw_AccX,time_cal);
    ts_raw_AccY = resample(ts_raw_AccY,time_cal);
    ts_raw_AccZ = resample(ts_raw_AccZ,time_cal);
    
    
    
    
    
    acc_raw_X = [];
    acc_raw_Y = [];
    acc_raw_Z = [];
    acc_calib_X = [];
    acc_calib_Y = [];
    acc_calib_Z = [];
    
    for i = 1:size(t,1)
        acc_raw_X = [acc_raw_X;ts_raw_AccX.Data(t(i,1):t(i,2))];
        acc_raw_Y = [acc_raw_Y;ts_raw_AccY.Data(t(i,1):t(i,2))];
        acc_raw_Z = [acc_raw_Z;ts_raw_AccZ.Data(t(i,1):t(i,2))];
        
        acc_calib_X = [acc_calib_X; data.IMU_CalData.Acc_calb.X(t(i,1):t(i,2))];
        acc_calib_Y = [acc_calib_Y; data.IMU_CalData.Acc_calb.Y(t(i,1):t(i,2))];
        acc_calib_Z = [acc_calib_Z; data.IMU_CalData.Acc_calb.Z(t(i,1):t(i,2))];
        
    end
    
    
    %{
%% Using the whole dataset
acc_raw_X = ts_raw_AccX.Data;
acc_raw_Y = ts_raw_AccY.Data;
acc_raw_Z = ts_raw_AccZ.Data;

acc_raw_X(1) = acc_raw_X(2);
acc_raw_Y(1) = acc_raw_Y(2);
acc_raw_Z(1) = acc_raw_Z(2);

acc_raw_X(end) = acc_raw_X(end-1);
acc_raw_Y(end) = acc_raw_Y(end-1);
acc_raw_Z(end) = acc_raw_Z(end-1);

acc_calib_X = data.IMU_CalData.Acc_calb.X;
acc_calib_Y = data.IMU_CalData.Acc_calb.Y;
acc_calib_Z = data.IMU_CalData.Acc_calb.Z;
    %}
    
    st_index = 1;
    end_index = size(acc_raw_X,1);
    
    measurement = [acc_raw_X(st_index:end_index) acc_raw_Y(st_index:end_index) acc_raw_Z(st_index:end_index) acc_calib_X(st_index:end_index) acc_calib_Y(st_index:end_index) acc_calib_Z(st_index:end_index)];
    X0 = [100,100,100,100,100,100];
    options = optimoptions('lsqnonlin','Display','iter','UseParallel',true,...
        'Algorithm','levenberg-marquardt');
    [X,resnorm,residual,exitflag,output] = lsqnonlin(@(X)calc_error(X,measurement),X0);
    X_acc = X
    
    corr_acc_X = acc_raw_X*X(1) + X(2);
    corr_acc_Y = acc_raw_Y*X(3) + X(4);
    corr_acc_Z = acc_raw_Z*X(5) + X(6);
    
    eAX = corr_acc_X - acc_calib_X;
    mAX = mean(eAX);
    eAY = corr_acc_Y - acc_calib_Y;
    mAY = mean(eAY);
    eAZ = corr_acc_Z - acc_calib_Z;
    mAZ = mean(eAZ);
    [mAX mAY mAZ]
    [std(eAX) std(eAY) std(eAZ)]
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555
%% Gyroscope bias

if(gyro_mode)
    %% Resampling raw data to the calibrated time
    ts_raw_GyroX = timeseries(data.IMU_RawData.Gyro.X,time_raw);
    ts_raw_GyroY = timeseries(data.IMU_RawData.Gyro.Y,time_raw);
    ts_raw_GyroZ = timeseries(data.IMU_RawData.Gyro.Z,time_raw);
    
    %ts_cal_GyroX = timeseries(data.IMU_CalData.Omega.Roll_V,time_cal);
    %ts_cal_GyroY = timeseries(data.IMU_CalData.Omega.Pitch_V,time_cal);
    %ts_cal_GyroZ = timeseries(data.IMU_CalData.Omega.Yaw_V,time_cal);
    
    ts_raw_GyroX = resample(ts_raw_GyroX,time_cal);
    ts_raw_GyroY = resample(ts_raw_GyroY,time_cal);
    ts_raw_GyroZ = resample(ts_raw_GyroZ,time_cal);
    
    
    
    
    gyro_raw_X = [];
    gyro_raw_Y = [];
    gyro_raw_Z = [];
    gyro_calib_X = [];
    gyro_calib_Y = [];
    gyro_calib_Z = [];
    
    for i = 1:size(t,1)
        gyro_raw_X = [gyro_raw_X;ts_raw_GyroX.Data(t(i,1):t(i,2))];
        gyro_raw_Y = [gyro_raw_Y;ts_raw_GyroY.Data(t(i,1):t(i,2))];
        gyro_raw_Z = [gyro_raw_Z;ts_raw_GyroZ.Data(t(i,1):t(i,2))];
        
        gyro_calib_X = [gyro_calib_X; data.IMU_CalData.Omega.Roll_V(t(i,1):t(i,2))];
        gyro_calib_Y = [gyro_calib_Y; data.IMU_CalData.Omega.Pitch_V(t(i,1):t(i,2))];
        gyro_calib_Z = [gyro_calib_Z; data.IMU_CalData.Omega.Yaw_V(t(i,1):t(i,2))];
    end
    
    
    %{
    %% Using the whole dataset
    gyro_raw_X = ts_raw_GyroX.Data;
    gyro_raw_Y = ts_raw_GyroY.Data;
    gyro_raw_Z = ts_raw_GyroZ.Data;
    
    gyro_raw_X(1) = gyro_raw_X(2);
    gyro_raw_Y(1) = gyro_raw_Y(2);
    gyro_raw_Z(1) = gyro_raw_Z(2);
    
    gyro_raw_X(end) = gyro_raw_X(end-1);
    gyro_raw_Y(end) = gyro_raw_Y(end-1);
    gyro_raw_Z(end) = gyro_raw_Z(end-1);
    
    gyro_calib_X = data.IMU_CalData.Omega.Roll_V;
    gyro_calib_Y = data.IMU_CalData.Omega.Pitch_V;
    gyro_calib_Z = data.IMU_CalData.Omega.Yaw_V;
    %}
    
    st_index = 1;
    end_index = 1000%size(gyro_raw_X,1);
    
    measurement = [gyro_raw_X(st_index:end_index) gyro_raw_Y(st_index:end_index) gyro_raw_Z(st_index:end_index) gyro_calib_X(st_index:end_index) gyro_calib_Y(st_index:end_index) gyro_calib_Z(st_index:end_index)];
    X0 = [100,100,100,100,100,100];
    options = optimoptions('lsqnonlin','Display','iter','UseParallel',true,...
        'Algorithm','levenberg-marquardt');
    [X,resnorm,residual,exitflag,output] = lsqnonlin(@(X)calc_error(X,measurement),X0);
    X_gyro = X
    
    corr_gyro_X = gyro_raw_X*X(1) + X(2);
    corr_gyro_Y = gyro_raw_Y*X(3) + X(4);
    corr_gyro_Z = gyro_raw_Z*X(5) + X(6);
    
    eGX = corr_gyro_X - gyro_calib_X;
    mGX = mean(eGX);
    eGY = corr_gyro_Y - gyro_calib_Y;
    mGY = mean(eGY);
    eGZ = corr_gyro_Z - gyro_calib_Z;
    mGZ = mean(eGZ);
    [mGX mGY mGZ]
    [std(eGX) std(eGY) std(eGZ)]
end

if(mag_mode)
    % Magnetometer readings
    ts_raw_Hx = timeseries(data.IMU_RawData.Mag.X,time_raw);
    ts_raw_Hy = timeseries(data.IMU_RawData.Mag.Y,time_raw);
    ts_raw_Hz = timeseries(data.IMU_RawData.Mag.Z,time_raw);
    
    ts_raw_Hx = resample(ts_raw_Hx,time_cal);
    ts_raw_Hy = resample(ts_raw_Hy,time_cal);
    ts_raw_Hz = resample(ts_raw_Hz,time_cal);
    
    mag_raw_Hx = [];
    mag_raw_Hy = [];
    mag_raw_Hz = [];
    mag_cal_Hx = [];
    mag_cal_Hy = [];
    mag_cal_Hz = [];
    
    for i = 1:size(t,1)
        mag_raw_Hx = [mag_raw_Hx; ts_raw_Hx.Data(t(i,1):t(i,2))];
        mag_raw_Hy = [mag_raw_Hy; ts_raw_Hy.Data(t(i,1):t(i,2))];
        mag_raw_Hz = [mag_raw_Hz; ts_raw_Hz.Data(t(i,1):t(i,2))];
        
        mag_cal_Hx = [mag_cal_Hx; data.IMU_CalData.Mag.Hx(t(i,1):t(i,2))];
        mag_cal_Hy = [mag_cal_Hy; data.IMU_CalData.Mag.Hy(t(i,1):t(i,2))];
        mag_cal_Hz = [mag_cal_Hz; data.IMU_CalData.Mag.Hz(t(i,1):t(i,2))];
    end
    
    %{
%% Using the whole dataset
mag_raw_Hx = ts_raw_Hx.Data;
mag_raw_Hy = ts_raw_Hy.Data;
mag_raw_Hz = ts_raw_Hz.Data;

mag_raw_Hx(1) = acc_raw_Hx(2);
mag_raw_Hy(1) = acc_raw_Hy(2);
mag_raw_Hz(1) = acc_raw_Hz(2);

mag_raw_Hx(end) = acc_raw_Hx(end-1);
mag_raw_Hy(end) = acc_raw_Hy(end-1);
mag_raw_Hz(end) = acc_raw_Hz(end-1);

mag_cal_Hx = data.IMU_CalData.Mag.Hx;
mag_cal_Hy = data.IMU_CalData.Mag.Hy;
mag_cal_Hz = data.IMU_CalData.Mag.Hz;
    %}
    
    st_index = 1;
    end_index = size(mag_raw_Hx,1);
    
    measurement = [mag_raw_Hx(st_index:end_index) mag_raw_Hy(st_index:end_index) mag_raw_Hz(st_index:end_index) mag_cal_Hx(st_index:end_index) mag_cal_Hy(st_index:end_index) mag_cal_Hz(st_index:end_index)];
    X0 = [100,100,100,100,100,100];
    options = optimoptions('lsqnonlin','Display','iter','UseParallel',true,...
        'Algorithm','levenberg-marquardt');
    [X,resnorm,residual,exitflag,output] = lsqnonlin(@(X)calc_error(X,measurement),X0);
    X_mag = X
    
    corr_mag_X = mag_raw_Hx*X(1) + X(2);
    corr_mag_Y = mag_raw_Hy*X(3) + X(4);
    corr_mag_Z = mag_raw_Hz*X(5) + X(6);
    
    eHX = corr_mag_X - mag_cal_Hx;
    mHX = mean(eHX);
    eHY = corr_mag_Y - mag_cal_Hy;
    mHY = mean(eHY);
    eHZ = corr_mag_Z - mag_cal_Hz;
    mHZ = mean(eHZ);
    [mHX mHY mHZ]
    [std(eHX) std(eHY) std(eHZ)]
end

