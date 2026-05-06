%% 本文件用于测试2关节串联机器人在CDPRs给定位姿下在工作空间的每个点的最大负载
% 串联2连杆机械臂有关参数如下，其工作空间是个半径为2m的圆
clc;
clear;
%% 
mp=50; %动平台的质量
m1=5; %关节1的质量
m2=10;  %关节2的质量
ml=0;  %负载的质量
q1=0;  %关节1与平台坐标系x轴夹角 0-360
q2=0;  %关节2与关节1的夹角 0-360
L1=1;  %连杆1的长度
L2=1;  %连杆2的长度
S=0.5; %连杆质心离近平台点的偏移量
g=9.81; %重力加速度
zb=[0 0 1]'; %框架坐标系的z轴单位向量
xp=[3 -1 3]'; %动平台的位置
theta=[60 -60 60]'; %动平台的夹角 XYZ欧拉角
x=theta(1);
y=theta(2);
z=theta(3);
Rxyz=Theta2RotationMatrix_Func(x,y,z);
x_vec=[];
y_vec=[];
z_vec=[];
%循环计算最大负载
for q1=0:2:360
    for q2=0:2:180
        ml_max=0;
        xGp=[0 -0.075 0]'; %动平台COM相对于动平台的原点的位置偏移
        xG1=[0.5*cosd(q1) 0.5*sind(q1) -0.125]';
        xG2=[cosd(q1)+0.5*cosd(q1+q2) sind(q1)+0.5*sind(q1+q2) -0.25]';
        xGl=[cosd(q1)+cosd(q1+q2) sind(q1)+sind(q1+q2) -0.25]';
        xGp=Rxyz*xGp;
        xG1=Rxyz*xG1;
        xG2=Rxyz*xG2;
        xGl=Rxyz*xGl;
        fp=-mp*g*zb;
        Tp=-mp*g*cross(xGp,zb);
        f1=-m1*g*zb;
        T1=-m1*g*cross(xG1,zb);
        f2=-m2*g*zb;
        T2=-m2*g*cross(xG2,zb);
        fl=-ml*g*zb;
        Tl=-ml*g*cross(xGl,zb);
        wp=[fp;Tp];
        w1=[f1;T1];
        w2=[f2;T2];
        wl=[fl;Tl];
        we=wp+wl+w1+w2;
        while PM_Method_Func(xp,-we,Rxyz)
            ml_max=ml_max+5; %每次加5kg质量
            fl=-ml_max*g*zb;
            Tl=-ml_max*g*cross(xGl,zb);
            wl=[fl;Tl];
            we=wp+wl+w1+w2;
        end
        x_vec=[x_vec,cosd(q1)+cosd(q1+q2)];
        y_vec=[y_vec,sind(q1)+sind(q1+q2)];
        z_vec=[z_vec,ml_max];
    end
end


%绘制二维散点图
scatter(x_vec,y_vec,10,z_vec,'filled') %filled表示点是实心点，缺省则为空心点
xlabel('x轴/m')
ylabel('y轴/m')
grid on
h = colorbar;
set(get(h,'label'),'string','最大负载(kg)');%给颜色栏命名
xlim([-2 2]) %设置坐标轴刻度取值范围
ylim([-2 2])
hold on;