%% 本文件用于计算CDPRs索力分布的正确性，跑一下力可行空间
% CDORs框架的长宽高：15m*11m*6m
% 由于框架坐标系的原点位于底部中心位置，所以x:-7.5m-7.5m y:-5.5m-5.5m z=0m-6m,动平台本身为1m3的长方体
% 截取工作空间内8*6*3的工作空间
clc;
clear;
%% 
mp=50; %动平台的质量
g=9.81; %重力加速度
xp=[0 0 0]';
theta=[60 -60 60]'; %动平台的夹角 XYZ欧拉角
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

tm=10;
tM=1000;

x_use_vec=[];
y_use_vec=[];
z_use_vec=[];
x_unuse_vec=[];
y_unuse_vec=[];
z_unuse_vec=[];
t1_vec=[];
t2_vec=[];
t3_vec=[];
t4_vec=[];
t5_vec=[];
t6_vec=[];
t7_vec=[];
t8_vec=[];
use_count=0;
unuse_count=0;
% 循环求解力可行空间
for x=-7:0.2:7 %防止动平台超出框架
    for y=-5:0.2:5
        for z=0.5:0.5:5
            xp=[x y z]';
            [t,bool]=Tension_Distribution4_Func(xp,theta,wp);
            if bool==1
                x_use_vec=[x_use_vec,x];
                y_use_vec=[y_use_vec,y];
                z_use_vec=[z_use_vec,z];
                t1_vec=[t1_vec,t(1,1)];
                t2_vec=[t2_vec,t(2,1)];
                t3_vec=[t3_vec,t(3,1)];
                t4_vec=[t4_vec,t(4,1)];
                t5_vec=[t5_vec,t(5,1)];
                t6_vec=[t6_vec,t(6,1)];
                t7_vec=[t7_vec,t(7,1)];
                t8_vec=[t8_vec,t(8,1)];
                use_count=use_count+1;
            else
                x_unuse_vec=[x_unuse_vec,x];
                y_unuse_vec=[y_unuse_vec,y];
                z_unuse_vec=[z_unuse_vec,z];
                unuse_count=unuse_count+1;
            end
        end
    end
end

precent=use_count/(use_count+unuse_count)

% figure
% scatter3(x_vec,y_vec,z_vec,10,t1_vec,'filled') %filled表示点是实心点，缺省则为空心点
% xlabel('x轴/m')
% ylabel('y轴/m')
% zlabel('z轴/m')
% grid on
% h = colorbar;
% set(get(h,'label'),'string','伸缩张力大小(N)');%给颜色栏命名
% xlim([-4 4]) %设置坐标轴刻度取值范围
% ylim([-3 3])
% hold on;

figure
scatter3(x_use_vec,y_use_vec,z_use_vec,'filled')
hold on
scatter3(x_unuse_vec,y_unuse_vec,z_unuse_vec,[],[1,0.5,0],'filled')
hold on
title('CDPRs力可行空间仿真分析(由索力分布算法)');
% legend('力可行域','力不可行域');
xlabel('x轴/m');
ylabel('y轴/m');
zlabel('z轴/m');

% figure
% scatter3(x_use_vec,y_use_vec,z_use_vec,'filled')
% hold on
% title('CDPRs力可行空间仿真分析(由索力分布算法)');
% legend('力可行域');
% xlabel('x轴/m');
% ylabel('y轴/m');
% zlabel('z轴/m');