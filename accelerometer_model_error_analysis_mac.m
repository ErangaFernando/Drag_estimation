% this code analyses the validity of different accelerometer models using
% the EuroC dataset


clear
clc
close all

addpath('quaternion');

% set dataset folder
% datasetPath = ...
%     '\\psf\Home\Desktop\Proj_VINS\ASL-EUROC\machine_hall1_easy';

datasetPath = ...
     '/Users/oscar/Desktop/Proj_VINS/ASL-EUROC/vicon_room1_medium';
dataset = dataset_load(datasetPath);

%Frame definitions visualization
figure();
title('sensor setup');
q_plotPose_mod([0 0 0], [1 0 0 0], 'W', 0.5);
p_WB_W=dataset.body{1}.sensor{5}.data.p_RS_R(:,1);
q_WB=dataset.body{1}.sensor{5}.data.q_RS(:,1);
R_WB=quat2rotm(q_WB');
q_plotPose_mod(p_WB_W, q_WB, 'B', 0.3);
T_BV=dataset.body{1}.sensor{6}.T_BS; %vicon frame
p_BV_B=T_BV(1:3,4);
R_BV=T_BV(1:3,1:3);
p_WV_W=p_WB_W + R_WB*p_BV_B;
R_WV=R_WB*R_BV;
%q_plotPose_mod(p_WV_W, rotm2quat(R_WV)', 'VICON', 0.5);
% we assume the COG is at the same level as the IMU frame and set the
% navigation frame there (see figure)

p_WN_W=p_WV_W;
p_WN_W(3)=p_WB_W(3);
R_WN=R_WV;
p_NB_W=-p_WN_W+p_WB_W;
p_NB_N=R_WN'*p_NB_W;
q_plotPose_mod(p_WN_W, rotm2quat(R_WN)', 'NAV', 0.5);

T_BC1=dataset.body{1}.sensor{1}.T_BS; %cam1 frame
p_BC1_B=T_BC1(1:3,4);
R_BC1=T_BC1(1:3,1:3);
p_WC1_W=p_WB_W + R_WB*p_BC1_B;
R_WC1=R_WB*R_BC1;
%q_plotPose_mod(p_WC1_W, rotm2quat(R_WC1)', 'C1', 0.3);
axis equal

acc_WB_B=dataset.body{1}.sensor{3}.data.a(:,:);
temp=[lowpass(acc_WB_B(1,:),0.2,200);
lowpass(acc_WB_B(2,:),0.2,200);
lowpass(acc_WB_B(3,:),0.2,200)];
acc_WB_B=temp;
omega_WB_B=dataset.body{1}.sensor{3}.data.a(:,:);

gt_offset=round((double(dataset.body{1}.sensor{5}.data.t(1))-double(dataset.body{1}.sensor{3}.data.t(1)))*1e-9/0.005);
gt_size=size(dataset.body{1}.sensor{5}.data.t,2);
n=size(dataset.body{1}.sensor{3}.data.t,2)

q_WB=[zeros(4,gt_offset) dataset.body{1}.sensor{5}.data.q_RS(:,:) zeros(4,n-gt_size-gt_offset)];
v_WB_W=[zeros(3,gt_offset) dataset.body{1}.sensor{5}.data.v_RS_R(:,:) zeros(3,n-gt_size-gt_offset)];

ba=mean(dataset.body{1}.sensor{5}.data.ba_S')';
bw=mean(dataset.body{1}.sensor{5}.data.bw_S')';


%% model 1
g_e=[0 0 -9.81]';
for i=1:n
    R_WB=quat2rotm(q_WB(:,i)');
    y_acc(:,i)=-R_WB'*g_e ;
    err1(:,i)=y_acc(:,i)-acc_WB_B(:,i);
end

figure()
subplot(3,1,1)
plot(acc_WB_B(1,:))
hold on
plot(y_acc(1,:),'-r')
title(['Model 1 prediction error =' num2str(norm(err1(1,2000:16000)))])

subplot(3,1,2)
plot(acc_WB_B(2,:))
hold on
plot(y_acc(2,:),'-r')
title(['Model 1 prediction error =' num2str(norm(err1(2,2000:16000)))])

subplot(3,1,3)
plot(acc_WB_B(3,:))
hold on
plot(y_acc(3,:),'-r')
title(['Model 1 prediction error =' num2str(norm(err1(3,2000:16000)))])

%% model 2
g_e=[0 0 -9.81]';

%optimization run
% dx=0.3;
% dy=0.3;
% R_BN=R_BV;
% param_vec0=[dx dy];
% reg_cost=registration_cost_rotor_drag(param_vec0,acc_WB_B(:,2000:16000),q_WB(:,2000:16000),v_WB_W(:,2000:16000),R_BN,ba); 
% options = optimset('Display','iter','MaxIter',10,'MaxFunEvals',5000,'Tolfun',1e-20,'TolX',1e-20);
% param_vec = lsqnonlin(@registration_cost_rotor_drag,param_vec0,[],[],options,acc_WB_B(:,2000:16000),q_WB(:,2000:16000),v_WB_W(:,2000:16000),R_BN,ba);
% reg_cost=registration_cost_rotor_drag(param_vec0,acc_WB_B(:,2000:16000),q_WB(:,2000:16000),v_WB_W(:,2000:16000),R_BN,ba); 
% param_vec  %gave 0.2for both

for i=1:n
    R_WB=quat2rotm(q_WB(:,i)');
    R_BN=R_BV;
    R_WN=R_WB*R_BN;
    D=diag([0.2 0.2 0]);
    y_acc2(:,i)=R_BN*(-D*R_WN'*v_WB_W(:,i)-g_e)+ba;
    err2(:,i)=y_acc2(:,i)-acc_WB_B(:,i);
end

figure()
subplot(3,1,1)
plot(acc_WB_B(1,:))
hold on
plot(y_acc2(1,:),'-r')
title(['Model 2 prediction error =' num2str(norm(err2(1,2000:16000)))]) %87

subplot(3,1,2)
plot(acc_WB_B(2,:))
hold on
plot(y_acc2(2,:),'-r')
title(['Model 2 prediction error =' num2str(norm(err2(2,2000:16000)))]) %12

subplot(3,1,3)
plot(acc_WB_B(3,:))
hold on
plot(y_acc2(3,:),'-r')
title(['Model 2 prediction error =' num2str(norm(err2(3,2000:16000)))]) %35


%% Model 3  Adding the rw^2 terms to acceleration and rw terms to velocity
% %rw term has insignificant effect
% %rw^2 term messes p more! (w^2 noise copling
% % we cannot ad the r alpha terms without differentiation of omega
% 
% for i=1:n
%     R_WB=quat2rotm(q_WB(:,i)');
%     R_BN=R_BV;
%     R_WN=R_WB*R_BN;
%     D=diag([0.2 0.2 0]);
%     v_WN_W=v_WB_W(:,i)-R_WN*skew(R_BN'*(omega_WB_B(:,i)-bw))*p_NB_N; %not working
%     v_WN_W=v_WB_W(:,i); %simplification
%     
%     y_acc2(:,i)=R_BN*(-D*R_WN'*v_WN_W+[0 0 9.81]')+ba +R_BN*skew(R_BN'*(omega_WB_B(:,i)-bw))*skew(R_BN'*(omega_WB_B(:,i)-bw))*p_NB_N;
%     err2(:,i)=y_acc2(:,i)-acc_WB_B(:,i);
% end
% 
% figure()
% subplot(3,1,1)
% plot(acc_WB_B(1,:))
% hold on
% plot(y_acc2(1,:),'-r')
% title(['Model 2 prediction error =' num2str(norm(err2(1,2000:16000)))]) %87
% 
% subplot(3,1,2)
% plot(acc_WB_B(2,:))
% hold on
% plot(y_acc2(2,:),'-r')
% title(['Model 2 prediction error =' num2str(norm(err2(2,2000:16000)))]) %12
% 
% subplot(3,1,3)
% plot(acc_WB_B(3,:))
% hold on
% plot(y_acc2(3,:),'-r')
% title(['Model 2 prediction error =' num2str(norm(err2(3,2000:16000)))]) %35
% 








