% %% This script will read the rosbag and extract data
%clear all;
%close all;
%clc;
% Reading the ROS bag
%filename = 'quadflight20180908_2speeds.bag';
%fn = 'quadflight20180908_2speeds.mat';

bag = rosbag(filename);

start_time = bag.StartTime;
end_time = bag.EndTime;
blocks = ceil((end_time-start_time)/2)
step = round((end_time - start_time)/blocks);
disp(['Rosbag ' filename ' is imported'])

% Tag IDs
dw1 = '0xC6B3';
dw2 = '0x97A5';
dw3 = '0x5028';
dw4 = '0x4C29';

%% Initializing the variables to store topics
topics =  bag.AvailableTopics.Properties.RowNames

imu_raw_available = 0;
imu_cal_available = 0;
optitrack_available = 0;
ctrl_available = 0;
joy_available = 0;
dw_available = 0;

% Final Data structure
data = struct('DataSet',filename);

for i = 1:size(topics,1)
    txt = topics{i}
    temp = bag.AvailableTopics{i,1}
    if(strcmp(txt,'/asctec/IMU_RAWDATA'))
        imu_raw_available = 1;
        imu_rawdata = struct('ROSTime',zeros(temp,1),'Time',struct('sec',zeros(temp,1),'nsec',zeros(temp,1)),'Seq',zeros(temp,1), ...
            'Gyro',struct('X',zeros(temp,1),'Y',zeros(temp,1),'Z',zeros(temp,1)), ...
            'Acc',struct('X',zeros(temp,1),'Y',zeros(temp,1),'Z',zeros(temp,1)), ...
            'Mag',struct('X',zeros(temp,1),'Y',zeros(temp,1),'Z',zeros(temp,1)))
        data.IMU_RawData = [];
    end
        if(strcmp(txt,'/asctec/IMU_CALCDATA'))
            imu_cal_available = 1;
            imu_caldata = struct('ROSTime',zeros(temp,1),'Time',struct('sec',zeros(temp,1),'nsec',zeros(temp,1)),'Seq',zeros(temp,1), ...
                'Angle',struct('Roll',zeros(temp,1),'Pitch',zeros(temp,1),'Yaw',zeros(temp,1)), ...
                'Omega',struct('Roll_V',zeros(temp,1),'Pitch_V',zeros(temp,1),'Yaw_V',zeros(temp,1)), ...
                'Acc_calb',struct('X',zeros(temp,1),'Y',zeros(temp,1),'Z',zeros(temp,1)), ...
                'Acc',struct('X',zeros(temp,1),'Y',zeros(temp,1),'Z',zeros(temp,1)), ...
                'Acc_angle',struct('Roll',zeros(temp,1),'Pitch',zeros(temp,1)), ...
                'Acc_abs',zeros(temp,1), ...
                'Mag',struct('Hx',zeros(temp,1),'Hy',zeros(temp,1),'Hz',zeros(temp,1)), ...
                'Mag_heading', zeros(temp,1), ...
                'Speed',struct('X',zeros(temp,1),'Y',zeros(temp,1),'Z',zeros(temp,1)), ...
                'Height',struct('Height',zeros(temp,1),'Dheight',zeros(temp,1),'HeightReference',zeros(temp,1),'DheightReference',zeros(temp,1)))
            data.IMU_CalData = [];
        end
    
    
        if(strcmp(txt,'/asctec1/pose'))
            optitrack_available = 1;
            optitrack_pose_t = struct('ROSTime',zeros(temp,1), ...
                'Position',struct('X',zeros(temp,1),'Y',zeros(temp,1),'Z',zeros(temp,1)), ...
                'Orientation',struct('X',zeros(temp,1),'Y',zeros(temp,1),'Z',zeros(temp,1),'W',zeros(temp,1)))
            data.Optitrack_Pose = [];
        end
    
        if(strcmp(txt,'/mav/cmd_pitch'))
            ctrl_available = 1;
            ctrl_commands = struct('ROSTime',zeros(temp,1), ...
                'Control',struct('roll',zeros(temp,1),'pitch',zeros(temp,1),'yaw',zeros(temp,1),'thrust',zeros(temp,1)))
            data.Control = [];
        end
        if(strcmp(txt,'/joy'))
            joy_available = 1;
        end
    
    if(strcmp(txt,'/dw_serial'))
        serial_data = struct('ROSTime',[],'Data',[]);
        %serial_data.Data = cell(temp,1);
        dw_available = 1;
        data.DW_data = struct('DW1',[],'DW2',[],'DW3',[],'DW4',[]);
    end
end


imu_l = 0;
imuCal_l = 0;
pose_l = 0;
joy_l = 0;
roll_l = 0;
pitch_l = 0;
yaw_l = 0;
thrust_l = 0;
dw_l = 0;

if(imu_raw_available)
    for h = 1:blocks
        % Use each topic to select a specific bag to extract the data
        %tic
        bagselect_imu = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','asctec/IMU_RAWDATA');
        imu = readMessages(bagselect_imu);
        for i = 1:size(imu,1)
            imu_rawdata.ROSTime(imu_l + i) = bagselect_imu.MessageList{i,1};
            imu_rawdata.Time.sec(imu_l + i) = imu{i}.Header.Stamp.Sec;
            imu_rawdata.Time.nsec(imu_l + i) = imu{i}.Header.Stamp.Nsec;
            imu_rawdata.Seq(imu_l + i) = imu{i}.Header.Seq;
            imu_rawdata.Gyro.X(imu_l + i) = imu{i}.GyroX;
            imu_rawdata.Gyro.Y(imu_l + i) = imu{i}.GyroY;
            imu_rawdata.Gyro.Z(imu_l + i) = imu{i}.GyroZ;
            imu_rawdata.Acc.X(imu_l + i) = imu{i}.AccX;
            imu_rawdata.Acc.Y(imu_l + i) = imu{i}.AccY;
            imu_rawdata.Acc.Z(imu_l + i) = imu{i}.AccZ;
            imu_rawdata.Mag.X(imu_l + i) = imu{i}.MagX;
            imu_rawdata.Mag.Y(imu_l + i) = imu{i}.MagY;
            imu_rawdata.Mag.Z(imu_l + i) = imu{i}.MagZ;
        end
        imu_l = imu_l + size(imu,1);
        %disp(['IMU RAW Block ' int2str(h) ' extracted'])
        %toc
    end
    data.IMU_RawData = imu_rawdata;
    disp('IMU Raw Data Extracted')
    clear functions
    clear bagselect_imu imu
end

if(imu_cal_available)
    for h = 1:blocks
        %tic
        bagselect_imu_cal = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','asctec/IMU_CALCDATA');
        imu_cal = readMessages(bagselect_imu_cal);
        for i = 1:size(imu_cal,1)
            imu_caldata.ROSTime(imuCal_l + i) = double( bagselect_imu_cal.MessageList{i,1});
            imu_caldata.Time.sec(imuCal_l + i) = double(imu_cal{i}.Header.Stamp.Sec);
            imu_caldata.Time.nsec(imuCal_l + i) = double(imu_cal{i}.Header.Stamp.Nsec);
            imu_caldata.Seq(imuCal_l + i) = double(imu_cal{i}.Header.Seq);
            imu_caldata.Angle.Roll(imuCal_l + i) = double(imu_cal{i}.AngleRoll)/1000;
            imu_caldata.Angle.Pitch(imuCal_l + i) = double(imu_cal{i}.AngleNick)/1000;
            imu_caldata.Angle.Yaw(imuCal_l + i) = double(imu_cal{i}.AngleYaw)/1000;
            imu_caldata.Omega.Roll_V(imuCal_l + i) = double(imu_cal{i}.AngvelRoll)*0.0154;
            imu_caldata.Omega.Pitch_V(imuCal_l + i) = double(imu_cal{i}.AngvelNick)*0.0154;
            imu_caldata.Omega.Yaw_V(imuCal_l + i) = double(imu_cal{i}.AngvelYaw)*0.0154;
            imu_caldata.Acc_calb.X(imuCal_l + i) = double(imu_cal{i}.AccXCalib)/10000;
            imu_caldata.Acc_calb.Y(imuCal_l + i) = double(imu_cal{i}.AccYCalib)/10000;
            imu_caldata.Acc_calb.Z(imuCal_l + i) = double(imu_cal{i}.AccZCalib)/10000;
            imu_caldata.Acc.X(imuCal_l + i) = double(imu_cal{i}.AccX)/10000;
            imu_caldata.Acc.Y(imuCal_l + i) = double(imu_cal{i}.AccY)/10000;
            imu_caldata.Acc.Z(imuCal_l + i) = double(imu_cal{i}.AccZ)/10000;
            imu_caldata.Acc_angle.Roll(imuCal_l + i) = double(imu_cal{i}.AccAngleRoll)/1000;
            imu_caldata.Acc_angle.Pitch(imuCal_l + i) = double(imu_cal{i}.AccAngleNick)/1000;
            imu_caldata.Acc_abs(imuCal_l + i) = double(imu_cal{i}.AccAbsoluteValue)/10000;
            imu_caldata.Mag.Hx(imuCal_l + i) = double(imu_cal{i}.Hx);
            imu_caldata.Mag.Hy(imuCal_l + i) = double(imu_cal{i}.Hy);
            imu_caldata.Mag.Hz(imuCal_l + i) = double(imu_cal{i}.Hz);
            imu_caldata.Mag_heading(imuCal_l + i) = double(imu_cal{i}.MagHeading)/1000;
            imu_caldata.Speed.X(imuCal_l + i) = double(imu_cal{i}.SpeedX);
            imu_caldata.Speed.Y(imuCal_l + i) = double(imu_cal{i}.SpeedY);
            imu_caldata.Speed.Z(imuCal_l + i) = double(imu_cal{i}.SpeedZ);
            imu_caldata.Height.Height(imuCal_l + i) = double(imu_cal{i}.Height)/1000;
            imu_caldata.Height.Dheight(imuCal_l + i) = double(imu_cal{i}.Dheight)/1000;
            imu_caldata.Height.HeightReference(imuCal_l + i) = double(imu_cal{i}.HeightReference)/1000;
            imu_caldata.Height.DheightReference(imuCal_l + i) = double(imu_cal{i}.DheightReference)/1000;
        end
        imuCal_l = imuCal_l+ size(imu_cal,1);
        %disp(['IMU CALIB Block ' int2str(h) ' extracted'])
        %toc
    end
    data.IMU_CalData = imu_caldata;
    disp('IMU Calibrated Data Extracted')
    clear functions
    clear bagselect_imu_cal imu_cal
end

if(optitrack_available)
    actual_size = 0;
    prev_time = 0;
    for h = 1:blocks
        %tic
        bagselect_pose = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','asctec1/pose');
        pose = readMessages(bagselect_pose);
        size(pose);
        for i = 1:1:size(pose,1)
            if(abs(bagselect_pose.MessageList{i,1} - prev_time) > 0.006)
                actual_size = actual_size + 1;
                optitrack_pose_t.ROSTime(actual_size) = bagselect_pose.MessageList{i,1};
                optitrack_pose_t.Position.X(actual_size) = pose{i}.Position.X;
                optitrack_pose_t.Position.Y(actual_size) = pose{i}.Position.Y;
                optitrack_pose_t.Position.Z(actual_size) = pose{i}.Position.Z;
                optitrack_pose_t.Orientation.X(actual_size) = pose{i}.Orientation.X;
                optitrack_pose_t.Orientation.Y(actual_size) = pose{i}.Orientation.Y;
                optitrack_pose_t.Orientation.Z(actual_size) = pose{i}.Orientation.Z;
                optitrack_pose_t.Orientation.W(actual_size) = pose{i}.Orientation.W;
            end
            prev_time = bagselect_pose.MessageList{i,1};
        end
        pose_l = pose_l + size(pose,1);
        %disp(['Pose Block ' int2str(h) ' extracted'])
        %toc
    end
    %%%*****************************************************
    % Checking for double messages
    temp = actual_size;
    optitrack_pose = struct('ROSTime',zeros(temp,1),'Position',struct('X',zeros(temp,1),'Y',zeros(temp,1),'Z',zeros(temp,1)),'Orientation',struct('X',zeros(temp,1),'Y',zeros(temp,1),'Z',zeros(temp,1),'W',zeros(temp,1)));
    for i = 1:temp
        optitrack_pose.ROSTime(i) = optitrack_pose_t.ROSTime(i);
        optitrack_pose.Position.X(i) = optitrack_pose_t.Position.X(i);
        optitrack_pose.Position.Y(i) = optitrack_pose_t.Position.Y(i);
        optitrack_pose.Position.Z(i) = optitrack_pose_t.Position.Z(i);
        optitrack_pose.Orientation.X(i) = optitrack_pose_t.Orientation.X(i);
        optitrack_pose.Orientation.Y(i) = optitrack_pose_t.Orientation.Y(i);
        optitrack_pose.Orientation.Z(i) = optitrack_pose_t.Orientation.Z(i);
        optitrack_pose.Orientation.W(i) = optitrack_pose_t.Orientation.W(i);
    end
    %%%*****************************************************
    data.Optitrack_Pose = optitrack_pose;
    disp('Optitrack Data Extracted')
    clear functions
    clear bagselect_pose pose
end

if(0)
    for h = 1:blocks
        %tic
        bagselect_joy = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','joy');
        joy = readMessages(bagselect_joy);
        joys%tic(joy_l+1:joy_l + size(joy,1),1) = joy;
        joy_l = joy_l + size(joy,1);
        %disp(['Block ' int2str(h) ' extracted'])
        
        %toc
    end
    clear functions
    clear bagselect_joy joy
end

if(ctrl_available)
    for h = 1:blocks
        %tic
        bagselect_roll = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','/mav/cmd_roll');
        roll = readMessages(bagselect_roll);
        for i = 1:size(roll,1)
            ctrl_commands.ROSTime(roll_l + i) = bagselect_roll.MessageList{i,1};
            ctrl_commands.Control.roll(roll_l + i) = roll{i}.Data;
        end
        roll_l = roll_l + size(roll,1);
        %disp(['Roll Block ' int2str(h) ' extracted'])
        %toc
    end
    clear bagselect_roll roll
    
    for h = 1:blocks
        %tic
        bagselect_pitch = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','/mav/cmd_pitch');
        pitch = readMessages(bagselect_pitch);
        for i = 1:size(pitch,1)
            ctrl_commands.ROSTime(pitch_l + i) = bagselect_pitch.MessageList{i,1};
            ctrl_commands.Control.pitch(pitch_l + i) = pitch{i}.Data;
        end
        pitch_l = pitch_l + size(pitch,1);
        %disp(['Pitch Block ' int2str(h) ' extracted'])
        %toc
    end
    clear bagselect_pitch pitch
    
    for h = 1:blocks
        %tic
        bagselect_yaw = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','/mav/cmd_yaw');
        yaw = readMessages(bagselect_yaw);
        for i = 1:size(yaw,1)
            ctrl_commands.ROSTime(yaw_l + i) = bagselect_yaw.MessageList{i,1};
            ctrl_commands.Control.yaw(yaw_l + i) = yaw{i}.Data;
        end
        yaw_l = yaw_l + size(yaw,1);
        %disp(['Yaw Block ' int2str(h) ' extracted'])
        %toc
    end
    clear bagselect_yaw yaw
    
    for h = 1:blocks
        %tic
        bagselect_thrust = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','/mav/cmd_thrust');
        thrust = readMessages(bagselect_thrust);
        for i = 1:size(thrust,1)
            ctrl_commands.ROSTime(thrust_l + i) = bagselect_thrust.MessageList{i,1};
            ctrl_commands.Control.thrust(thrust_l + i) = thrust{i}.Data;
        end
        thrust_l = thrust_l + size(thrust,1);
        %disp(['Thrust Block ' int2str(h) ' extracted'])
        %toc
    end
    clear bagselect_thrust thrust
    data.Control = ctrl_commands;
disp('Control Data Extracted')
end

if(dw_available)
    for h = 1:blocks
        %tic
        bagselect_dw = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','/dw_serial');
        dw_data = readMessages(bagselect_dw);
        temp_val = 1;
        for i = 1:size(dw_data,1)
            if(dw_data{i}.Data(1) == 'T')
                str = cellstr(dw_data{i}.Data);
                serial_data.ROSTime = [serial_data.ROSTime;bagselect_dw.MessageList{i,1}];
                serial_data.Data = [serial_data.Data;str];
                temp_val = temp_val + 1;
            end
        end
        dw_l = dw_l + temp_val;
        %disp(['Block ' int2str(h) ' extracted'])
        %toc
    end
    
    %% Extracting the data from serial data
    % Creating strucutres for each beacon and tag
    DW_beacon_1 = struct('ID',dw1,'range',[],'time',[]);
    DW_beacon_2 = struct('ID',dw2,'range',[],'time',[]);
    DW_beacon_3 = struct('ID',dw3,'range',[],'time',[]);
    DW_beacon_4 = struct('ID',dw4,'range',[],'time',[]);
    
    DWbeacons = [DW_beacon_1,DW_beacon_2,DW_beacon_3,DW_beacon_4];
    no_beacons = size(DWbeacons,2);
    
    DW_tag_1 = struct('ID','TAG1','pos',[],'time',[]);
    i = 1;
    for k = 1:size(serial_data.Data,1)
        tline = serial_data.Data{k};
        if ~ischar(tline)
            break;
        end
        % Get the indices of the spaces and brackets
        spaces = regexp(tline,'[ ]');
        lbrackets = regexp(tline,'[\[]');
        rbrackets = regexp(tline,'[\]]');
        sets = size(spaces,2);
        % Read the time
        if(sets>0)
            time = str2double(tline(3:(spaces(1)-1)));
        else
            time = str2double(tline(3:(size(tline,2))));
        end
        DW_tag_1.time = [DW_tag_1.time;time];
        for k = 1:no_beacons
            DWbeacons(k).time = [DWbeacons(k).time;time];
        end
        % Read the positions and the  distances
        if (sets > 0)
            pos_avail = 0;
            for j = 1:sets
                if strcmp('POS',tline((spaces(j)+1):(spaces(j)+3)))
                    pos = tline(lbrackets(j)+1 : rbrackets(j)-1);
                    pos = strsplit(pos,',');
                    DW_tag_1.pos = [DW_tag_1.pos;str2double(pos)];
                    pos_avail = 1;
                elseif strcmp('DIST',tline((spaces(j)+1):(spaces(j)+4))) && pos_avail == 1
                    if((2*j) < size(rbrackets,2))
                        dist = tline(lbrackets(2*j - 1)+1 : rbrackets(2*j - 1)-1);
                        dist = strsplit(dist,',');
                        tagID =  tline((spaces(j)+7):(spaces(j)+12));
                        
                        for k = 1:no_beacons
                            if strcmp(tagID,DWbeacons(k).ID)
                                DWbeacons(k).range = [DWbeacons(k).range;str2double(dist)];
                            end
                        end
                    end
                elseif strcmp('DIST',tline((spaces(j)+1):(spaces(j)+4))) && pos_avail == 0
                    dist = tline(lbrackets(2*j)+1 : rbrackets(2*j)-1);
                    dist = strsplit(dist,',');
                    tagID =  tline((spaces(j)+7):(spaces(j)+12));
                    
                    for k = 1:no_beacons
                        if strcmp(tagID,DWbeacons(k).ID)
                            DWbeacons(k).range = [DWbeacons(k).range;str2double(dist)];
                        end
                    end
                end
            end
            % Fill the gaps
            length = size(DW_tag_1.time,1);
            if (size(DW_tag_1.pos,1) ~= length)
                DW_tag_1.pos = [DW_tag_1.pos;[-1000,-1000,-1000,-1000]];
            end
            for k = 1:no_beacons
                length = size(DWbeacons(k).time,1);
                if(size(DWbeacons(k).range,1) ~= length)
                    DWbeacons(k).range = [DWbeacons(k).range;[-1000,-1000]];
                end
            end
        end
        i = i+1;
    end
    % Remove noise and fill data
    for i = 1:3
        for k = 1:size(DWbeacons(i).range(:,1),1)
            if(DWbeacons(i).range(k,1) == -1000)
                DWbeacons(i).range(k,1) = (DWbeacons(i).range(k-1,1) + DWbeacons(i).range(k+1,1))/2;
            end
        end
        
    end
    data.DW_data.ROSTime = serial_data.ROSTime;
    data.DW_data.DW1 = DWbeacons(1);
    data.DW_data.DW2 = DWbeacons(2);
    data.DW_data.DW3 = DWbeacons(3);
    data.DW_data.DW4 = DWbeacons(4);
disp('Range Data Extracted')
end





clear imu_rawdata imu_caldata optitrack_pose ctrl_commands
save(fn,'data')
%{
    quad_stationary_2_edited.mat: Statinary sections
    1.  10:1100
    2.  1300:1750
    3.  1850:2000
    4.  2080:2280
    5.  2350:2800 : Quadrotor is ON
    6.  2900:3300
    7.  3400:3800
    8.  3950:4200 : Start changing yaw
    9.  4350:4600 : Changing yaw
    10. 4750:5100 : End of changing yaw
    11. 5200:5450
    12. 7700:8100
    13. 8400:9100
    14. 9450:10050
    15. 10200:11060


%}