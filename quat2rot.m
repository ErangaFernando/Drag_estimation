function C = quat2rot(q)
%% This function will return the rotation matrix corresponding to the quaternion rotaion
% Input : q : A Quaternion, can be either a column vector or a row vector
% Output: C : Rotation matrix

if(numel(q) ~= 4)
    error('Check dimensions of the quaternion');
end

if (size(q,2) == 4)
    q = q';
end

q_x = [   0  -q(3)  q(2);
        q(3)    0  -q(1);
       -q(2)  q(1)    0 ];

C = (2*q(4)^2 - 1)*eye(3) - 2*q(4)*q_x + 2*q(1:3,1)*q(1:3,1)';
%C = (q(4)^2 - q(1:3,1)'*q(1:3,1))*eye(3) - 2*q(4)*q_x + 2*q(1:3,1)*q(1:3,1)';
