function [sys,x0,str,ts,simStateCompliance] = Path_Planning_SFunc(t,x,u,flag)
%{
输入参数：1、xinit_x,
        2、xinit_y,
        3、xinit_z:    轨迹规划的初始点
        4、xend_x,
        5、xend_y,
        6、xend_z:     轨迹规划的终点
        7、vm:         限制的速度的最大值
        8、am:         限制的加速度最大值
        一共有8个输入参数
状态参数:1、xd_x,
        2、xd_y,
        3、xd_z:     规划出的当前时间的轨迹点
        4、vd:       规划出的当前速度
        5、ad:       规划处的当前加速度
        6、t_now     当前时间
        7、Condition 目前图像是梯形还是三角形 (1是梯形 0是三角形)
        一共有7个状态参数
输出参数:1、xd_x
        2、xd_y
        3、xd_z:     规划出的当前轨迹点
        4、xd_dot_x
        5、xd_dot_y
        6、xd_dot_z: 规划出的当前速度
        7、xd_ddot_x
        8、xd_ddot_y
        9、xd_ddot_z:规划出的当前加速度
        10、t_now:   当前时间
%}
switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  case 2,
    sys=mdlUpdate(t,x,u);

  case 3,
    sys=mdlOutputs(t,x,u);

  case {1,4,9},
    sys=[];

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag)); %异常处理

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;%用于设置模块参数的结构体用simsizes来生成
sizes.NumContStates  = 0;%模块连续状态变量的个数
sizes.NumDiscStates  = 7;%模块离散状态变量的个数
sizes.NumOutputs     = 10;%模块输出变量的个数
sizes.NumInputs      = 8;%模块输入变量的个数
sizes.DirFeedthrough = 1;%模块是否存在直接贯通，1表示存在直接贯通，若为0，则mdlOutputs函数里不能有u
sizes.NumSampleTimes = 1;%模块的采样时间个数,至少是一个
sys=simsizes(sizes);%设置完后赋给sys输出
x0=zeros(7,1);%系统状态变量设置
str=[];
% ts=[0.001 0];%采样周期设为0.001，该模块在仿真开始0s后就跑
ts=[0.001 0];%采样周期设为0.01，该模块在仿真开始0s后就跑
simStateCompliance = 'UnknownSimState';

function sys=mdlUpdate(t,x,u)
%     T=0.001; %采样时间为0.001，即采样频率为1kHz
    T=0.001;
    xinit_x=u(1);
    xinit_y=u(2);
    xinit_z=u(3);
    xend_x=u(4);
    xend_y=u(5);
    xend_z=u(6);
    Pf=sqrt((xend_x-xinit_x)^2+(xend_y-xinit_y)^2+(xend_z-xinit_z)^2); %两点之间的距离
    v_max=u(7);   %最大速度
    a_max=u(8);   %最大加速度
    l=[xend_x-xinit_x,xend_y-xinit_y,xend_z-xinit_z]';%两点之间的向量
    l_norm=l/norm(l);%两点之间的单位向量
    
    ta=v_max/a_max;     %加速和减速需要的时间
    Pa=0.5*a_max*ta^2;  %加速或减速产生的位置量 
    t_m=(Pf-2*Pa)/v_max;%最大速度需要的时间
    t_f=t_m+2*ta;       %到达目标位置所需要的时间
    t_now=x(6);
    
    if t_f-2*ta>0
        %达到最大速度，梯形
        x(7)=1;
    else
        % 未达到最大速度，速度曲线为三角形
        x(7)=0;
    end
    
    if x(7)==1
        if t_now<=ta
            x(4)=a_max*t_now;
            x(5)=a_max;
            pt=0.5*a_max*t_now*t_now;
            x(1)=xinit_x+l_norm(1)*pt;
            x(2)=xinit_y+l_norm(2)*pt;
            x(3)=xinit_z+l_norm(3)*pt;
        end
        if t_now>ta && t_now<=t_f-ta
            x(4)=v_max;
            x(5)=0;
            pt=0.5*a_max*ta*ta+a_max*ta*(t_now-ta);
            x(1)=xinit_x+l_norm(1)*pt;
            x(2)=xinit_y+l_norm(2)*pt;
            x(3)=xinit_z+l_norm(3)*pt;
        end
        if t_now>t_f-ta && t_now<=t_f
            x(4)=v_max-a_max*(t_now-(t_f-ta));
            x(5)=-a_max;
            pt=Pf-0.5*a_max*(t_f-t_now)^2;
            x(1)=xinit_x+l_norm(1)*pt;
            x(2)=xinit_y+l_norm(2)*pt;
            x(3)=xinit_z+l_norm(3)*pt;
        end
        if t_now>t_f
            x(1)=xend_x;
            x(2)=xend_y;
            x(3)=xend_z;
            x(4)=0;
            x(5)=0;
        end
    end
    
    if x(7)==0
        ta=sqrt(Pf/a_max);
        t_f=2*ta;
        if t_now<=ta
            x(4)=a_max*t_now;
            x(5)=a_max;
            pt=0.5*a_max*t_now*t_now;
            x(1)=xinit_x+l_norm(1)*pt;
            x(2)=xinit_y+l_norm(2)*pt;
            x(3)=xinit_z+l_norm(3)*pt;
        end
        if t_now>ta && t_now<=t_f
            x(4)=a_max*(t_f-t_now);
            x(5)=-a_max;
            pt=Pf-0.5*a_max*(t_f-t_now)^2;
            x(1)=xinit_x+l_norm(1)*pt;
            x(2)=xinit_y+l_norm(2)*pt;
            x(3)=xinit_z+l_norm(3)*pt;
        end
        if t_now>t_f
            x(1)=xend_x;
            x(2)=xend_y;
            x(3)=xend_z;
            x(4)=0;
            x(5)=0;
        end
    end
    
    x(6)=x(6)+T;
    
    sys = [x(1),x(2),x(3),x(4),x(5),x(6),x(7)];

function sys=mdlOutputs(t,x,u)

    xinit_x=u(1);
    xinit_y=u(2);
    xinit_z=u(3);
    xend_x=u(4);
    xend_y=u(5);
    xend_z=u(6);
    l=[xend_x-xinit_x,xend_y-xinit_y,xend_z-xinit_z]';%两点之间的向量
    l_norm=l./norm(l);%两点之间的单位向量
    
    if x(6)==0
        xd_x=u(1);
        xd_y=u(2);
        xd_z=u(3);
    else
        xd_x=x(1);
        xd_y=x(2);
        xd_z=x(3);
    end
    
    xd_dot_x=x(4)*l_norm(1);
    xd_dot_y=x(4)*l_norm(2);
    xd_dot_z=x(4)*l_norm(3);
    xd_ddot_x=x(5)*l_norm(1);
    xd_ddot_y=x(5)*l_norm(2);
    xd_ddot_z=x(5)*l_norm(3);
    t_now=x(6);
    sys = [xd_x,xd_y,xd_z,xd_dot_x,xd_dot_y,xd_dot_z,xd_ddot_x,xd_ddot_y,xd_ddot_z,t_now];
    