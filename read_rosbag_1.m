% %% This script will read the rosbag and extract data

clear all;
close all;
%clc;
% Reading the ROS bag
filename = 'quadflight20180822_1.bag';
bag = rosbag(filename);

start_time = bag.StartTime;
end_time = bag.EndTime;
blocks = ceil((end_time-start_time)/2)
step = round((end_time - start_time)/blocks);
disp(['Rosbag ' filename ' is imported'])


%% Initializing the variables to store topics
topics =  bag.AvailableTopics.Properties.RowNames

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RosTime = struct('sec',0);
Time = struct('sec',0,'nsec',0);
Gyro = struct('X',0,'Y',0,'Z',0);
Acc = struct('X',0,'Y',0,'Z',0);
Mag = struct('X',0,'Y',0,'Z',0);
position = struct('X',0,'Y',0,'Z',0);
orientation = struct('X',0,'Y',0,'Z',0,'W',0);
control = struct('roll',0,'pitch',0,'yaw',0,'thrust',0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
imu_rawdata = cell(bag.AvailableTopics{1,1},1);
optitract_pose = cell(bag.AvailableTopics{2,1},1);
joystic = cell(bag.AvailableTopics{3,1},1);
ctrl_commands = cell(bag.AvailableTopics{4,1},4);

imu_l = 0;
pose_l = 0;
joy_l = 0;
roll_l = 0;
pitch_l = 0;
yaw_l = 0;
thrust_l = 0;

for h = 1:blocks
	% Use each topic to select a specific bag to extract the data
	tic
	bagselect_imu = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','asctec/IMU_RAWDATA');
	imu = readMessages(bagselect_imu);
	imu_rawdata(imu_l+1:imu_l + size(imu,1),1) = imu;
	imu_l = imu_l + size(imu,1);
	disp(['Block ' int2str(h) ' extracted'])
	toc
end
clear functions
clear bagselect_imu imu

for h = 1:blocks
	tic
 	bagselect_pose = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','asctec1/pose');
 	pose = readMessages(bagselect_pose);
    toc
    tic
 	optitract_pose(pose_l+1:pose_l + size(pose,1),1) = pose;
 	pose_l = pose_l + size(pose,1);
 	disp(['Block ' int2str(h) ' extracted'])
 	toc
end
clear functions
clear bagselect_pose pose

for h = 1:blocks
	tic
 	bagselect_joy = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','joy');
 	joy = readMessages(bagselect_joy);
 	joystic(joy_l+1:joy_l + size(joy,1),1) = joy; 	
 	joy_l = joy_l + size(joy,1);
 	disp(['Block ' int2str(h) ' extracted'])
    
 	toc
end
clear functions
clear bagselect_joy joy

% for h = 1:blocks
% 	tic	
% 	bagselect_roll = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','/mav/cmd_roll');
% 	roll = readMessages(bagselect_roll);
% 	ctrl_commands(roll_l+1:roll_l + size(roll,1),1) = roll;
% 	roll_l = roll_l + size(roll,1);
% 	disp(['Block ' int2str(h) ' extracted'])
% 	toc
% end
% clear bagselect_roll roll
% 
% for h = 1:blocks
% 	tic
% 	bagselect_pitch = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','/mav/cmd_pitch');
% 	pitch = readMessages(bagselect_pitch);
% 	ctrl_commands(pitch_l+1:pitch_l + size(pitch,1),2) = pitch;
% 	pitch_l = pitch_l + size(pitch,1);
% 	disp(['Block ' int2str(h) ' extracted'])
% 	toc
% end
% clear bagselect_pitch pitch
% 
% for h = 1:blocks
% 	tic
% 	bagselect_yaw = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','/mav/cmd_yaw');
% 	yaw = readMessages(bagselect_yaw);
% 	ctrl_commands(yaw_l+1:yaw_l + size(yaw,1),3) = yaw;
% 	yaw_l = yaw_l + size(yaw,1);
% 	disp(['Block ' int2str(h) ' extracted'])
% 	toc
% end
% clear bagselect_yaw yaw
% 
% for h = 1:blocks
% 	tic
% 	bagselect_thrust = select(bag,'Time',[start_time+(h-1)*step start_time+h*step],'Topic','/mav/cmd_thrust');
% 	thrust = readMessages(bagselect_thrust);
% 	ctrl_commands(thrust_l+1:thrust_l + size(thrust,1),4) = thrust;
% 	thrust_l = thrust_l + size(thrust,1);	
% 	disp(['Block ' int2str(h) ' extracted'])
% 	toc
% end
% clear bagselect_thrust thrust


imuRAWDATA = struct('rosTime',zeros(100,1),'time',[struct('sec',0.0,'nsec',0.0)])