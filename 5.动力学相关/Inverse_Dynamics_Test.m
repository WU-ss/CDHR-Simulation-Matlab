clc;
clear;
% 当前位姿
p=[0 0 3]';
theta=[30 -30 30]';
% 线速度与线加速度
p_dot=[0 0 0]';
p_ddot=[0 0 0]';
% 角速度与角加速度
theta_dot=[10 10 10]';
theta_ddot=[10 10 10]';

f  = Inverse_Dynamics_Func(p,p_dot,p_ddot,theta,theta_dot,theta_ddot)

mp=58.63; %动平台的质量
g=9.81; %重力加速度
x=theta(1);
y=theta(2);
z=theta(3);
Rxyz=Theta2RotationMatrix_Func(x,y,z);
xGp=[0 -0.075 0]'; %动平台COM相对于动平台的原点的位置偏移
xGp=Rxyz*xGp; %考虑旋转的作用
zb=[0 0 1]'; %框架坐标系下z轴的单位向量
f=-mp*g*zb;
T=-mp*g*cross(xGp,zb);
wp = -[f;T]