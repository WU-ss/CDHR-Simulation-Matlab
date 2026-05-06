%% 本文件用于测试2关节串联机器人在CDPRs给定位姿下在工作空间的每个点的最大负载
% 串联2连杆机械臂有关参数如下，其工作空间是个半径为2m的圆
clc;
clear;
%% 
mp=79; %动平台的质量
m1=10; %关节1的质量
m2=5;  %关节2的质量
ml=0;  %负载的质量
q1=0;  %关节1与平台坐标系x轴夹角 0-360
q2=0;  %关节2与关节1的夹角 0-360
L1=1;  %连杆1的长度
L2=1;  %连杆2的长度
S=0.25; %连杆质心离近平台点的偏移量
g=9.81; %重力加速度

dq1=0;
dq2=0;
ddq1=0;
ddq2=0;
Ixx1=1;
Iyy1=1;
Izz1=1;
Ixx2=1;
Iyy2=1;
Izz2=1;
Ixx3=1;
Iyy3=1;
Izz3=1;

zb=[0 0 1]'; %框架坐标系的z轴单位向量
xp=[0 0 3]'; %动平台的位置
theta=[0 0 0]'; %动平台的夹角 XYZ欧拉角
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
        xGp=Rxyz*xGp;
        fp=-mp*g*zb;
        Tp=-mp*g*cross(xGp,zb);
        wp=[fp;Tp];
        [f11,n11] = Twolinks_Dynamic_Func(q1,q2,dq1,dq2,ddq1,ddq2,m1,m2,ml_max,L1,L2,Ixx1,Iyy1,Izz1,Ixx2,Iyy2,Izz2,Ixx3,Iyy3,Izz3,Rxyz);
        f11_rot=Rxyz*f11;
        n11_rot=Rxyz*n11;
        wl=[f11_rot;n11_rot];
        we=wp+wl;
        while PM_Method_Func(xp,-we,Rxyz)
            ml_max=ml_max+5; %每次加5kg质量
            [f11,n11] = Twolinks_Dynamic_Func(q1,q2,dq1,dq2,ddq1,ddq2,m1,m2,ml_max,L1,L2,Ixx1,Iyy1,Izz1,Ixx2,Iyy2,Izz2,Ixx3,Iyy3,Izz3,Rxyz);
            f11_rot=Rxyz*f11;
            n11_rot=Rxyz*n11;
            wl=[f11_rot;n11_rot];
            we=wp+wl;
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