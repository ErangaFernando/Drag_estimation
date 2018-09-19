function [ X ] = estimate_rpy_offset( data,st,ed)
%ESTIMATE_RPY This function will estimate the initial rotation from the
%optitrack to the Quadrotor frame
% data: ROS bag data
% length: length of the stationary period
% rpy : Roll, pitch and yaw of the quadrotor
% X : Return the rotation matrix


q_eo = quatnormalize([data.Optitrack_Pose.Orientation.X, data.Optitrack_Pose.Orientation.Y, data.Optitrack_Pose.Orientation.Z, data.Optitrack_Pose.Orientation.W]);
rpy = [data.IMU_CalData.Angle.Roll, data.IMU_CalData.Angle.Pitch, (data.IMU_CalData.Angle.Yaw)]*pi/180;
X0 = [10,10,10];

%% Rearrange the quaternions and interpolate the quaternions
k = 1;
new_quat = zeros(size(rpy,1),4);

index = zeros(size(rpy,1),1);
for i = 1:size(q_eo,1)
	time = data.IMU_CalData.ROSTime(k);
	if (data.Optitrack_Pose.ROSTime(i) > time)
		index(k) = i;
		k = k+1;
    end
    if k > size(rpy,1)
        break;
    end
end

for i = 1:size(index,1)
	in = index(i);
	if(in == 1)
		new_quat(i,:) = q_eo(i,:);
        4
    elseif (in == 0)
		new_quat(i,:) = q_eo(in+1,:);
        5
	else
		q = [q_eo(in,4), q_eo(in,1:3)];
		p = [q_eo(in-1,4), q_eo(in-1,1:3)];
		f = (data.IMU_CalData.ROSTime(i) - data.Optitrack_Pose.ROSTime(in-1))/(data.Optitrack_Pose.ROSTime(in) - data.Optitrack_Pose.ROSTime(in-1));        
		q_int = quatinterp(p,q,f,'slerp');
		new_quat(i,:) = [q_int(2:4) q_int(1)];
	end
end
disp('Remapping done');
rpy_opti = quat2eul([new_quat(:,4) new_quat(:,1:3)],'ZYX');
rpy_opti = [rpy_opti(:,3) -rpy_opti(:,2) -rpy_opti(:,1)];

options = optimoptions('lsqnonlin','Display','iter','UseParallel',true,...
                          'Algorithm','levenberg-marquardt');

[X,resnorm,residual,exitflag,output] = lsqnonlin(@(X)cal_err(X,rpy(st:ed,:),rpy_opti(st:ed,:)),X0);

offset = mean(rpy(st:ed,:)) - mean(rpy_opti(st:ed))
offset - X

vis = 0
if(vis == 1)
    figure
    plot(rpy(st:ed,1))
    hold on
    plot(rpy_opti(st:ed,1) + X(1));
    plot(ones(ed-st,1)*mean(rpy(st:ed,1)))
    figure
    plot(rpy(st:ed,2))
    hold on
    plot(rpy_opti(st:ed,2)+X(2));
    plot(ones(ed-st,1)*mean(rpy(st:ed,2)))
    figure
    plot(rpy(st:ed,3))
    hold on
    plot(rpy_opti(st:ed,3)+X(3));
    plot(ones(ed-st,1)*mean(rpy(st:ed,3)))
    
end

end

function [F] = cal_err(X,rpy,rpy_opti)
    F = rpy - rpy_opti - repmat(X,size(rpy,1),1);
% 	l = size(rpy,1);
% 	F = zeros(1,9*l);
%     for i = 1:l
%     	phi = rpy(i,1);
% 		theta = rpy(i,2);
% 		psi = rpy(i,3);
% 
% 		phi_op = rpy_opti(i,1);
% 		theta_op = rpy_opti(i,2);
% 		psi_op = rpy_opti(i,3);
% 
% 		R = [[ cos(psi + X(3))*cos(theta + X(2)), cos(psi + X(3))*sin(phi + X(1))*sin(theta + X(2)) - cos(phi + X(1))*sin(psi + X(3)), sin(phi + X(1))*sin(psi + X(3)) + cos(phi + X(1))*cos(psi + X(3))*sin(theta + X(2))]
%             [ cos(theta + X(2))*sin(psi + X(3)), cos(phi + X(1))*cos(psi + X(3)) + sin(phi + X(1))*sin(psi + X(3))*sin(theta + X(2)), cos(phi + X(1))*sin(psi + X(3))*sin(theta + X(2)) - cos(psi + X(3))*sin(phi + X(1))]
%             [         -sin(theta + X(2)),                              cos(theta + X(2))*sin(phi + X(1)),                              cos(phi + X(1))*cos(theta + X(2))]];
%     
% 		R_eb = [[ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)]
% 				[ cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)]
% 				[         -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta)]];
% 
%         Rq = [[ cos(psi_op)*cos(theta_op), cos(psi_op)*sin(phi_op)*sin(theta_op) - cos(phi_op)*sin(psi_op), sin(phi_op)*sin(psi_op) + cos(phi_op)*cos(psi_op)*sin(theta_op)]
%               [ cos(theta_op)*sin(psi_op), cos(phi_op)*cos(psi_op) + sin(phi_op)*sin(psi_op)*sin(theta_op), cos(phi_op)*sin(psi_op)*sin(theta_op) - cos(psi_op)*sin(phi_op)]
%               [         -sin(theta_op),                              cos(theta_op)*sin(phi_op),                              cos(phi_op)*cos(theta_op)]];
% 
%         error = R - R_eb'*Rq;        
%         F(9*(i-1) + 1 : 9*i) = reshape(error,1,9);
%         
% 
%     end
    
end

