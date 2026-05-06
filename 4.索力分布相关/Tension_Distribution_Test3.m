%% 主要测试Tension_Distribution4_Func函数，以便后续做成MATLAB Function
clc;
clear;

mp=79; %动平台的质量
g=9.81; %重力加速度

p=[2 2 2]';
theta=[0 0 0]'; %动平台的夹角 XYZ欧拉角

x=theta(1);
y=theta(2);
z=theta(3);
Rxyz=Theta2RotationMatrix_Func(x,y,z);
xGp=[0 -0.075 0]'; %动平台COM相对于动平台的原点的位置偏移
% xGp=[0 0 0]'; %动平台COM相对于动平台的原点的位置偏移
xGp=Rxyz*xGp; %考虑旋转的作用
zb=[0 0 1]'; %框架坐标系下z轴的单位向量
f=-mp*g*zb;
T=-mp*g*cross(xGp,zb);
wp=-[f;T];
f=wp;

[t,bool]=Tension_Distribution4_Func(p,theta,f);