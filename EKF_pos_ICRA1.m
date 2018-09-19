function DAQ_EKF = EKF_pos_ICRA1(filename,st,ed)
%% This function will run the estimator for the flight data recorded for ICRA
%
% Input : filename : Data file saved for the flight
% Output: DAQ_all : Data from simulator and estimator
%
% This EKF uses the exact model of the ETH paper with z direction aero
% dynamic force and z direction acceleration. Thrust rate is taken as an input 
% IMU update will be done depending on the time
% 
%close all;
clearvars -except filename st ed

%% State vector
% X     : X position in world frame
% Y     : Y position in world frame
% Z     : Z position in world frame
% roll  : Roll angle in current orientation estimate
% pitch : Pitch angle in current orientation estimate
% yaw   : Yaw angle in current orientation estimate
% B_gx  : Bias in X axis gyroscope
% B_gy  : Bias in Y axis gyroscope
% B_gz  : Bias in Z axis gyroscope
% v_x   : X velocity component of quadrotor in body frame
% v_y   : Y velocity component of quadrotor in body frame
% v_z   : Z velocity component of quadrotor in body frame
% T     : Thrust
% b_ax  : Bias in X accelerometer measurement
% b_ay  : Bias in Y accelerometer measurement
% b_az  : Bias in Z accelerometer measurement

model_no = 1
sys_params;
load(filename);
%EKF_m_ICRA(data,model_no);

%st = 400%400%50;
%ed = 1900%1600%1050;
data = data_resample(data);
DAQ_EKF = EKF_m_ICRA(data,model_no,st,ed)

end

function DAQ_EKF = EKF_m_ICRA(data,model_no,st,ed)
    global tau ge m
    global anchor_pos
    global sigma_U4 sigma_prop sigma_gx sigma_gy sigma_gz sigma_bgx sigma_bgy sigma_bgz sigma_mx sigma_my sigma_mz sigma_bax sigma_bay sigma_baz
    global sigma_ax sigma_ay sigma_az sigma_zh sigma_r

    if(st == 0 || ed == 0)
        st = 1;
        ed = size(data.IMU_CalData.ROSTime,1);
    end
    length = ed-st+1;
    start_time = data.IMU_CalData.ROSTime(st);
    end_time = data.IMU_CalData.ROSTime(ed);
    
    for i = 1:size(data.Optitrack_Pose.ROSTime,1)
        if(data.Optitrack_Pose.ROSTime(i,1) > start_time)
            X = data.Optitrack_Pose.Position.X(i-1) 
            Y = -data.Optitrack_Pose.Position.Y(i-1) 
            Z = -data.Optitrack_Pose.Position.Z(i-1)
            break;
        end
    end

    for i = 1:size(data.DW_data.ROSTime,1)
        if(data.DW_data.ROSTime(i) > start_time)
            range_id = i-1
            break
        end
    end

    %% Initializing
    %% Initializing the state vector
    
    roll  = 0; pitch = 0; yaw   = 0;
    b_gx  = 0; b_gy  = 0; b_gz  = 0;
    v_x   = 0; v_y   = 0; v_z   = 0;
    b_ax  = 0; b_ay  = 0; b_az  = 0;

    k_prop = 6.41e-6 - 0*6.41e-6;
    k_pp = 0.00011 - 0*0.00011;
    k_pa = 0.00023 - 0*0.00023;

    U_nominal = m*ge;
    prop_spd = 0.045/k_pp;


    %% Executing EKF: Initialization
    
    no_states = 14;
    cov_mat = 0.03*eye(no_states);
    % Initializing the state vector
    state_vec = zeros(no_states,1);
    state_vec(11,1) = m*ge;
    state_vec(1,1) = X;
    state_vec(2,1) = Y;
    state_vec(3,1) = Z;
    state_vec'
    X_ba_hist = zeros(no_states,length);
    state_vec_hist = zeros(no_states,length);
    po_cov_hist = zeros(no_states,length);
    pr_cov_hist = zeros(no_states,length);
    rank_hist = zeros(1,length);      

    prev_t = start_time;

    r1 = sqrt((data.Optitrack_Pose.Position.X - anchor_pos(1,1)).^2 + (-data.Optitrack_Pose.Position.Y - anchor_pos(1,2)).^2 + (-data.Optitrack_Pose.Position.Z - anchor_pos(1,3)).^2);
    r2 = sqrt((data.Optitrack_Pose.Position.X - anchor_pos(2,1)).^2 + (-data.Optitrack_Pose.Position.Y - anchor_pos(2,2)).^2 + (-data.Optitrack_Pose.Position.Z - anchor_pos(2,3)).^2);
    r3 = sqrt((data.Optitrack_Pose.Position.X - anchor_pos(3,1)).^2 + (-data.Optitrack_Pose.Position.Y - anchor_pos(3,2)).^2 + (-data.Optitrack_Pose.Position.Z - anchor_pos(3,3)).^2);
    dist = [r1 r2 r3];
    for i = st+1:ed
        % Time
        t = data.IMU_CalData.ROSTime(i);
        dt = t - prev_t;
        prev_t = t;
        % Data for the estimation
        % Inputs
        g_x = data.IMU_CalData.Omega.Roll_V(i)*pi/180; g_y = data.IMU_CalData.Omega.Pitch_V(i)*pi/180; g_z =0; data.IMU_CalData.Omega.Yaw_V(i)*pi/180;
        control_in = (data.Control.Control.thrust(i+1) - data.Control.Control.thrust(i))/(data.IMU_CalData.ROSTime(i+1)-t);
        % Measurements
        a_x = data.IMU_CalData.Acc_calb.X(i)*ge; a_y = data.IMU_CalData.Acc_calb.Y(i)*ge; a_z = data.IMU_CalData.Acc_calb.Z(i)*ge;
        range_time = data.DW_data.ROSTime(range_id);
        range_id;
        r1 = data.DW_data.DW1.range(range_id,1);
        r2 = data.DW_data.DW2.range(range_id,1);
        r3 = data.DW_data.DW3.range(range_id,1);

        %% Noise vectors
        W = diag([sigma_gx, sigma_gy, sigma_bgx, sigma_bgy, sigma_mx, sigma_my, sigma_mz, sigma_U4, sigma_bax, sigma_bay, sigma_baz]);
        R1 = diag([sigma_ax, sigma_ay, sigma_az]);
        if (model_no == 1)
            R2 = diag([sigma_r, sigma_r, sigma_r]);
        end
        if(model_no == 2)
            R2 = diag([sigma_r, sigma_r]);
        end
        if(model_no == 3)
            R2 = diag([sigma_r]);
        end

        %% Prediction
        phi   = state_vec(4); % Roll angle
        theta = state_vec(5); % Pitch angle
        psi   = 0;%state_vec(6); % Yaw angle is assumed to be zero always
        b_gx  = state_vec(6); % Bias in X gyro
        b_gy  = state_vec(7); % Bias in Y gyro
        %b_gz  = state_vec(9); % Bias in Z gyro
        v_x   = state_vec(8); % Velocity in X direction
        v_y   = state_vec(9); % Velocity in Y direction
        v_z   = state_vec(10); % Velocity in Z direction
        T     = state_vec(11); % Thrust
        b_ax  = state_vec(12); % Bias in X accelerometer
        b_ay  = state_vec(13); % Bias in Y accelerometer
        b_az  = state_vec(14); % Bias in Z accelerometer

        U4 = control_in;

        f_xdot = [  v_z*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - v_y*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + v_x*cos(psi)*cos(theta);
 v_y*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - v_z*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + v_x*cos(theta)*sin(psi);
                                                                        v_z*cos(phi)*cos(theta) - v_x*sin(theta) + v_y*cos(theta)*sin(phi);
                                                          g_x - b_gx - cos(phi)*tan(theta)*(b_gz - g_z) - sin(phi)*tan(theta)*(b_gy - g_y);
                                                                                             sin(phi)*(b_gz - g_z) - cos(phi)*(b_gy - g_y);
                                                                                                                                 -b_gx/tau;
                                                                                                                                 -b_gy/tau;
                                                                                                   - ge*sin(theta) - (k_pp*prop_spd*v_x)/m;
                                                                                            ge*cos(theta)*sin(phi) - (k_pp*prop_spd*v_y)/m;
                                                                                      ge*cos(phi)*cos(theta) - T/m - (k_pa*prop_spd*v_z)/m;
                                                                                                                                        U4;
                                                                                                                                         0;
                                                                                                                                         0;
                                                                                                                                         0];

        % Square integration
        x_ba = state_vec + dt*f_xdot;
        %ba = x_ba'
        % log prediction state vector
        X_ba_hist(:,i) = x_ba;

        %% Jacobian of non-linear function
        F = [[ 0, 0, 0,   v_y*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + v_z*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)), v_z*cos(phi)*cos(psi)*cos(theta) - v_x*cos(psi)*sin(theta) + v_y*cos(psi)*cos(theta)*sin(phi),      0,                    0,                    0, cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi),    0, 0, 0, 0];
[ 0, 0, 0, - v_y*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - v_z*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), v_z*cos(phi)*cos(theta)*sin(psi) - v_x*sin(psi)*sin(theta) + v_y*cos(theta)*sin(phi)*sin(psi),      0,                    0,                    0, cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta),    0, 0, 0, 0];
[ 0, 0, 0,                                                                 v_y*cos(phi)*cos(theta) - v_z*cos(theta)*sin(phi),                          - v_x*cos(theta) - v_z*cos(phi)*sin(theta) - v_y*sin(phi)*sin(theta),      0,                    0,                    0,         -sin(theta),                              cos(theta)*sin(phi),    0, 0, 0, 0];
[ 0, 0, 0,                                               sin(phi)*tan(theta)*(b_gz - g_z) - cos(phi)*tan(theta)*(b_gy - g_y),         - cos(phi)*(b_gz - g_z)*(tan(theta)^2 + 1) - sin(phi)*(b_gy - g_y)*(tan(theta)^2 + 1),     -1, -sin(phi)*tan(theta), -cos(phi)*tan(theta),                   0,                                                0,    0, 0, 0, 0];
[ 0, 0, 0,                                                                     cos(phi)*(b_gz - g_z) + sin(phi)*(b_gy - g_y),                                                                                             0,      0,            -cos(phi),             sin(phi),                   0,                                                0,    0, 0, 0, 0];
[ 0, 0, 0,                                                                                                                 0,                                                                                             0, -1/tau,                    0,                    0,                   0,                                                0,    0, 0, 0, 0];
[ 0, 0, 0,                                                                                                                 0,                                                                                             0,      0,               -1/tau,                    0,                   0,                                                0,    0, 0, 0, 0];
[ 0, 0, 0,                                                                                                                 0,                                                                                -ge*cos(theta),      0,                    0,                    0,  -(k_pp*prop_spd)/m,                                                0,    0, 0, 0, 0];
[ 0, 0, 0,                                                                                            ge*cos(phi)*cos(theta),                                                                       -ge*sin(phi)*sin(theta),      0,                    0,                    0,                   0,                               -(k_pp*prop_spd)/m,    0, 0, 0, 0];
[ 0, 0, 0,                                                                                           -ge*cos(theta)*sin(phi),                                                                       -ge*cos(phi)*sin(theta),      0,                    0,                    0,                   0,                                                0, -1/m, 0, 0, 0];
[ 0, 0, 0,                                                                                                                 0,                                                                                             0,      0,                    0,                    0,                   0,                                                0,    0, 0, 0, 0];
[ 0, 0, 0,                                                                                                                 0,                                                                                             0,      0,                    0,                    0,                   0,                                                0,    0, 0, 0, 0];
[ 0, 0, 0,                                                                                                                 0,                                                                                             0,      0,                    0,                    0,                   0,                                                0,    0, 0, 0, 0];
[ 0, 0, 0,                                                                                                                 0,                                                                                             0,      0,                    0,                    0,                   0,                                                0,    0, 0, 0, 0]];


        %% Weighing matrix Qd
        G = [[ 0,                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
[ 0,                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
[ 0,                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
[ 1, sin(phi)*tan(theta), 0, 0, 0, 0, 0, 0, 0, 0, 0];
[ 0,            cos(phi), 0, 0, 0, 0, 0, 0, 0, 0, 0];
[ 0,                   0, 1, 0, 0, 0, 0, 0, 0, 0, 0];
[ 0,                   0, 0, 1, 0, 0, 0, 0, 0, 0, 0];
[ 0,                   0, 0, 0, 1, 0, 0, 0, 0, 0, 0];
[ 0,                   0, 0, 0, 0, 1, 0, 0, 0, 0, 0];
[ 0,                   0, 0, 0, 0, 0, 1, 0, 0, 0, 0];
[ 0,                   0, 0, 0, 0, 0, 0, 1, 0, 0, 0];
[ 0,                   0, 0, 0, 0, 0, 0, 0, 1, 0, 0];
[ 0,                   0, 0, 0, 0, 0, 0, 0, 0, 1, 0];
[ 0,                   0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
        Q_d = G*W*G'*dt*dt;

        %% Taylor series approximation
        A = eye(no_states) + F*dt;
         
        %% Calculating priori covariance matrix P_min
        P_min = A*cov_mat*A' + Q_d;
        % Log priori covariance matrix P_min
        pr_cov_hist(:,i) = diag(P_min);

        %% Measurement update - Serial update     
        phi   = x_ba(4);
        theta = x_ba(5);
        %pai   = x_ba(6);
        v_x   = x_ba(8); % Velocity in X direction
        v_y   = x_ba(9); % Velocity in Y direction
        v_z   = x_ba(10); % Velocity in Y direction
        T     = x_ba(11);% Thrust
        % Anchor positions
        px = anchor_pos(:,1);
        py = anchor_pos(:,2);
        pz = anchor_pos(:,3);
        % Jacobian
        H1 = [[                                                                 0,                                                                 0,                                                                 0,                       0,          ge*cos(theta), 0, 0, 0, -(k_pp*prop_spd)/m,                  0,    0, 1, 0, 0];
[                                                                 0,                                                                 0,                                                                 0, -ge*cos(phi)*cos(theta), ge*sin(phi)*sin(theta), 0, 0, 0,                  0, -(k_pp*prop_spd)/m,    0, 0, 1, 0];
[                                                                 0,                                                                 0,                                                                 0,  ge*cos(theta)*sin(phi), ge*cos(phi)*sin(theta), 0, 0, 0,                  0,                  0, -1/m, 0, 0, 1]];

        % Kalman gain
        chol((H1*P_min*H1' + R1));
        K1_gain = P_min*H1'/(H1*P_min*H1' + R1);
        % Measurement prediction
        Y1_ba = [               b_ax +  0*ge*sin(theta) - (k_pp*prop_spd*v_x)/m;
                       b_ay  - 0*ge*cos(theta)*sin(phi) - (k_pp*prop_spd*v_y)/m;
                 b_az  - T/m - 0*ge*cos(phi)*cos(theta) - (k_pa*prop_spd*v_z)/m];
        % Actual measurement
        Y1 = [a_x;a_y;a_z];
        %Y1_ba'
        %Y1'
        % State vector update
        state_vec = x_ba + K1_gain*(Y1 - Y1_ba);
        % Posterior covariance matrix
        cov_mat = (eye(no_states) - K1_gain*H1)*P_min;

        if (i<ed)
            if(data.IMU_CalData.ROSTime(i+1) > range_time)
                %disp('Corrected');
                %[range_id i data.IMU_CalData.ROSTime(i)-range_time]
                if((r1~= -1000 && r2~= -1000 && r3~= -1000))
                x_ba = state_vec;
                P_min = cov_mat;
                % Variables
                Xe    = x_ba(1);
                Ye    = x_ba(2);
                Ze    = x_ba(3);
                %% Executing different measurement models depending on the scenario
                % Jacobian measurement vector
                H2 = [[ (2*Xe - 2*px(1))/(2*((Xe - px(1))^2 + (Ye - py(1))^2 + (Ze - pz(1))^2)^(1/2)), (2*Ye - 2*py(1))/(2*((Xe - px(1))^2 + (Ye - py(1))^2 + (Ze - pz(1))^2)^(1/2)), (2*Ze - 2*pz(1))/(2*((Xe - px(1))^2 + (Ye - py(1))^2 + (Ze - pz(1))^2)^(1/2)),                       0,                      0, 0, 0, 0,                  0,                  0,    0, 0, 0, 0];
[ (2*Xe - 2*px(2))/(2*((Xe - px(2))^2 + (Ye - py(2))^2 + (Ze - pz(2))^2)^(1/2)), (2*Ye - 2*py(2))/(2*((Xe - px(2))^2 + (Ye - py(2))^2 + (Ze - pz(2))^2)^(1/2)), (2*Ze - 2*pz(2))/(2*((Xe - px(2))^2 + (Ye - py(2))^2 + (Ze - pz(2))^2)^(1/2)),                       0,                      0, 0, 0, 0,                  0,                  0,    0, 0, 0, 0];
[ (2*Xe - 2*px(3))/(2*((Xe - px(3))^2 + (Ye - py(3))^2 + (Ze - pz(3))^2)^(1/2)), (2*Ye - 2*py(3))/(2*((Xe - px(3))^2 + (Ye - py(3))^2 + (Ze - pz(3))^2)^(1/2)), (2*Ze - 2*pz(3))/(2*((Xe - px(3))^2 + (Ye - py(3))^2 + (Ze - pz(3))^2)^(1/2)),                       0,                      0, 0, 0, 0,                  0,                  0,    0, 0, 0, 0]];
                % Kalman gain
                chol((H2*P_min*H2' + R2));
                K2_gain = P_min*H2'/(H2*P_min*H2' + R2);
                % Measurement prediction
                Y2_ba = [((Xe - px(1))^2 + (Ye - py(1))^2 + (Ze - pz(1))^2)^(1/2);
                         ((Xe - px(2))^2 + (Ye - py(2))^2 + (Ze - pz(2))^2)^(1/2);
                         ((Xe - px(3))^2 + (Ye - py(3))^2 + (Ze - pz(3))^2)^(1/2)];
                % Actual measurement from simulation
                Y2 = [r1; r2; r3];
                % State vector update
                state_vec = x_ba + K2_gain*(Y2 - Y2_ba);
                % Posterior covariance matrix
                cov_mat = (eye(no_states) - K2_gain*H2)*P_min;
                % Move to next measurement
                range_id = range_id + 1;
                
                else
                    range_id = range_id + 1;
                end
            end
        end
        %% Log
        % log estimation history
        state_vec_hist(:,i) = state_vec;
        %st = state_vec'
        % Log posterior covariance matrix P_plus 
        diag(cov_mat);
        po_cov_hist(:,i) = diag(cov_mat);
    end

    DAQ_EKF.info = {'state_vec - Corrected state vector';
                'X_ba      - Predicted state vector ';
                'pos_cov   - Posterior covariance matrix';
                'pri_cov   - Priori covariance matrix';
                'rank_hist - Rank at each time step'};
    DAQ_EKF.state_vec = state_vec_hist;
    DAQ_EKF.X_ba = X_ba_hist;
    DAQ_EKF.pos_cov = po_cov_hist;
    DAQ_EKF.pri_cov = pr_cov_hist;
    DAQ_EKF.rank = rank_hist;

    figure
    plot(data.IMU_CalData.ROSTime(1:ed),DAQ_EKF.state_vec(3,:))
    hold on
    plot(data.Optitrack_Pose.ROSTime,-data.Optitrack_Pose.Position.Z)

    figure;plot(data.IMU_CalData.ROSTime(1:ed),DAQ_EKF.state_vec(2,:))
    hold on
    plot(data.Optitrack_Pose.ROSTime,-data.Optitrack_Pose.Position.Y)
    
    figure;plot(data.IMU_CalData.ROSTime(1:ed),DAQ_EKF.state_vec(1,:))
    hold on
    plot(data.Optitrack_Pose.ROSTime,data.Optitrack_Pose.Position.X)
end

function data_out = data_resample(data);
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

    ts_C_X = resample(ts_C_X,dw_time);
    ts_C_Y = resample(ts_C_Y,dw_time);
    ts_C_Z = resample(ts_C_Z,dw_time);



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
end
