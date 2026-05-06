clc;
clear;
% 当前位姿
p=[0 0 3]';
theta=[0 0 0]';
% 需要转换的速度与角速度
p_dot=[1 0 3]';
theta_dot=[1 0 0]';

[l_dot,q_dot] = Jacobian_Func(p_dot,theta_dot,p,theta)

[p_dot,theta_dot] = Inverse_Jacobian_Func(q_dot,p,theta)