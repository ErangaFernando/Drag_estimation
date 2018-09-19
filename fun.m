function [ out ] = fun( X,a,b )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%b = reshape(b,1,numel(b));
r = X(1);
p = X(2);
y = X(3);

R = [cos(y)*cos(p), -sin(y)*cos(r) + cos(y)*sin(p)*sin(r), sin(y)*sin(r) + cos(y)*sin(p)*cos(r);
     sin(y)*cos(p), cos(y)*cos(r) + sin(y)*sin(p)*sin(r), -cos(y)*sin(r) + sin(y)*sin(p)*cos(r);
     -sin(p), cos(p)*sin(r), cos(p)*cos(r)];
f = (R*a) - b;

out = reshape(f,1,numel(f));
end

