function [l,l_len] = InverseKenematics_Func(xp,theta)
%UNTITLED3 求解CDPRs的逆运动学解
%   Detailed explanation goes here
x=theta(1);
y=theta(2);
z=theta(3);
Rxyz=Theta2RotationMatrix_Func(x,y,z);
%% CoGiRo的参数 8绳索6自由度的CDPRs
% ai的定义
a1=[-7.5 -5.5 6]';
a2=[-7.5 -5.5 6]';
a3=[-7.5 5.5 6]';
a4=[-7.5 5.5 6]';
a5=[7.5 5.5 6]';
a6=[7.5 5.5 6]';
a7=[7.5 -5.5 6]';
a8=[7.5 -5.5 6]';
% bi的定义
b1=[0.5 -0.5 0]';
b2=[-0.5 0.5 1]';
b3=[-0.5 -0.5 0]';
b4=[0.5 0.5 1]';
b5=[-0.5 0.5 0]';
b6=[0.5 -0.5 1]';
b7=[0.5 0.5 0]';
b8=[-0.5 -0.5 1]';
%% 计算li=ai-xp-Rbi
l1=a1-xp-Rxyz*b1;
l2=a2-xp-Rxyz*b2;
l3=a3-xp-Rxyz*b3;
l4=a4-xp-Rxyz*b4;
l5=a5-xp-Rxyz*b5;
l6=a6-xp-Rxyz*b6;
l7=a7-xp-Rxyz*b7;
l8=a8-xp-Rxyz*b8;
% 求绳索向量的模长
l1_len=norm(l1);
l2_len=norm(l2);
l3_len=norm(l3);
l4_len=norm(l4);
l5_len=norm(l5);
l6_len=norm(l6);
l7_len=norm(l7);
l8_len=norm(l8);
l=[l1 l2 l3 l4 l5 l6 l7 l8]';
l_len=[l1_len l2_len l3_len l4_len l5_len l6_len l7_len l8_len]';
end