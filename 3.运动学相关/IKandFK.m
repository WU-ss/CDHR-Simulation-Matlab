%% 本文件用于测试CDPRs的逆运动学与正运动学
% 首先生成CDPRs的运动轨迹,使用该轨迹位姿通过逆运动学得到绳索长度，然后使用该绳索长度通过正运动学求解出推测的运动轨迹
clc;
clear;
%% 生成动平台轨迹
t=(0:0.01:10)';
x=5*cos(pi/5*t);
y=5*sin(pi/5*t);
z=3*t/10+1;
theta=[0 0 0]';
l1_len_vec=[];
l2_len_vec=[];
l3_len_vec=[];
l4_len_vec=[];
l5_len_vec=[];
l6_len_vec=[];
l7_len_vec=[];
l8_len_vec=[];
%% 通过逆运动学，从位姿生成绳索长度向量
for i=1:size(t,1)
    xp=[x(i,1) y(i,1) z(i,1)]';
    [l,l_len]=InverseKenematics_Func(xp,theta);
    l1_len_vec=[l1_len_vec,l_len(1)];
    l2_len_vec=[l2_len_vec,l_len(2)];
    l3_len_vec=[l3_len_vec,l_len(3)];
    l4_len_vec=[l4_len_vec,l_len(4)];
    l5_len_vec=[l5_len_vec,l_len(5)];
    l6_len_vec=[l6_len_vec,l_len(6)];
    l7_len_vec=[l7_len_vec,l_len(7)];
    l8_len_vec=[l8_len_vec,l_len(8)];
end

figure
%绘制三维散点图
scatter3(x,y,z,10,t,'filled') %filled表示点是实心点，缺省则为空心点
xlabel('x轴/m')
ylabel('y轴/m')
zlabel('z轴/m')
grid on
title('函数生成的CDPRs位姿');
% legend('位姿xp')
h = colorbar;
set(get(h,'label'),'string','时间(s)');%给颜色栏命名
xlim([-7.5 7.5]) %设置坐标轴刻度取值范围
ylim([-5.5 5.5])
zlim([0 6])

figure
plot(t,l1_len_vec,'k');
hold on
plot(t,l2_len_vec,'b');
hold on
plot(t,l3_len_vec,'g');
hold on
plot(t,l4_len_vec,'c');
hold on
plot(t,l5_len_vec,'r');
hold on
plot(t,l6_len_vec,'m');
hold on
plot(t,l7_len_vec,'y');
hold on
plot(t,l8_len_vec,'color',[0.9290 0.6940 0.1250]);
title('CDPRs逆运动学仿真分析');
legend('l1','l2','l3','l4','l5','l6','l7','l8');
xlabel('时间t/s');
ylabel('绳索长度L/m');
grid on

%% 通过正运动学，从位姿生成绳索长度向量
xp_vec=[];
F_vec=[];
for i=1:size(t,1)
    L=[l1_len_vec(i) l2_len_vec(i) l3_len_vec(i) l4_len_vec(i) l5_len_vec(i) l6_len_vec(i) l7_len_vec(i) l8_len_vec(i)];
    [xp_fk,k,minF]=ForwardKenamatics_Func(L);
    xp_vec=[xp_vec,xp_fk];
    F_vec=[F_vec,minF];
end
figure
%绘制三维散点图
scatter3(xp_vec(1,:),xp_vec(2,:),xp_vec(3,:),10,F_vec,'filled') %filled表示点是实心点，缺省则为空心点
xlabel('x轴/m')
ylabel('y轴/m')
zlabel('z轴/m')
grid on
title('CDPRs正运动学求解的轨迹');
% legend('位姿xp from FK')
h = colorbar;
set(get(h,'label'),'string','误差总和F');%给颜色栏命名
xlim([-7.5 7.5]) %设置坐标轴刻度取值范围
ylim([-5.5 5.5])
zlim([0 6])
