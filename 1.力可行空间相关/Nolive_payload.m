%% 本文件用于计算CDPRs空载时的力可行空间
% CDORs框架的长宽高：15m*11m*6m
% 由于框架坐标系的原点位于底部中心位置，所以x:-7.5m-7.5m y:-5.5m-5.5m z=0.5m-5m,动平台本身为1m3的长方体
clc;
clear;
%% 
mp=50; %动平台的质量
g=9.81; %重力加速度
xp=[0 0 1]';
theta=[90 -90 90]'; %动平台的夹角 XYZ欧拉角
x=theta(1);
y=theta(2);
z=theta(3);
Rxyz=Theta2RotationMatrix_Func(x,y,z);
xGp=[0 -0.075 0]'; %动平台COM相对于动平台的原点的位置偏移
xGp=Rxyz*xGp; %考虑旋转的作用
zb=[0 0 1]'; %框架坐标系下z轴的单位向量

f=-mp*g*zb;
T=-mp*g*cross(xGp,zb);
wp=-[f;T];
x_use_vec=[];
y_use_vec=[];
z_use_vec=[];
x_unuse_vec=[];
y_unuse_vec=[];
z_unuse_vec=[];
use_count=0;
unuse_count=0;
precent=0;
% 循环求解力可行空间
for x=-7:0.2:7 %防止动平台超出框架
    for y=-5:0.2:5
        for z=0.5:0.5:5
            xp=[x y z]';
            if PM_Method_Func(xp,wp,Rxyz)==1
                x_use_vec=[x_use_vec,x];
                y_use_vec=[y_use_vec,y];
                z_use_vec=[z_use_vec,z];
                use_count=use_count+1;
            end
            if PM_Method_Func(xp,wp,Rxyz)==0
                x_unuse_vec=[x_unuse_vec,x];
                y_unuse_vec=[y_unuse_vec,y];
                z_unuse_vec=[z_unuse_vec,z];
                unuse_count=unuse_count+1;
            end
        end
    end
end

precent=use_count/(use_count+unuse_count)

figure
scatter3(x_use_vec,y_use_vec,z_use_vec,'filled')
hold on
scatter3(x_unuse_vec,y_unuse_vec,z_unuse_vec,[],[1,0.5,0],'filled')
hold on
title('CDPRs力可行空间仿真分析');
% legend('力可行域','力不可行域');
xlabel('x轴/m');
ylabel('y轴/m');
zlabel('z轴/m');

% figure
% scatter3(x_use_vec,y_use_vec,z_use_vec,'filled')
% hold on
% title('CDPRs力可行空间仿真分析');
% legend('力可行域');
% xlabel('x轴/m');
% ylabel('y轴/m');
% zlabel('z轴/m');