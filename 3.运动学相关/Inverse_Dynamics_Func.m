function [f] = Inverse_Dynamics_Func(p,p_dot,p_ddot,theta,theta_dot,theta_ddot)
% CDPRs的动力学求解
% 输入:p,p_dot,p_ddot,theta,theta_dot,theta_ddot 位置与姿态及其各阶导数
% 输出:f 动平台产生的wrench
mp=79;          %动平台质量
g=9.81;
cp=[0 -0.075 0]'; %动平台COM相对于动平台的原点的位置偏移
IG=[1 0 0;0 1 0;0 0 1]; %动平台相对于动平台坐标系的惯性矩阵

x_dot=[p_dot;theta_dot];
x_ddot=[p_ddot;theta_ddot];

%% 得到旋转矩阵
Rxyz=Theta2RotationMatrix_Func(theta(1),theta(2),theta(3));

%% 得到S(x),在得到平台在世界坐标系下的角速度omiga,进而得到omiga^是omiga对应的斜对称矩阵
thetax=theta(1);
thetay=theta(2);
thetaz=theta(3);

thetax_dot=theta_dot(1);
thetay_dot=theta_dot(2);
thetaz_dot=theta_dot(3);

Sx=[1 0 sind(thetay); 0 cosd(thetax) -sind(thetax)*cosd(thetay); 0 sind(thetax) cosd(thetax)*cosd(thetay)];
omiga=Sx*theta_dot;
v1=omiga(1);
v2=omiga(2);
v3=omiga(3);
omiga_sharp=[0 -v3 v2;v3 0 -v1;-v2 v1 0];
Sx=[eye(3,3) zeros(3,3); zeros(3,3) Sx];

%% 得到S_dot_(x,x_dot)的每个元素为S(x)相应位置元素的导数
Sx_dot_2_2=[0 0 cosd(thetay)*thetay_dot;0 -sind(thetax)*thetax_dot -cosd(thetax)*cosd(thetay)*thetax_dot+sind(thetax)*sind(thetay)*thetay_dot; 0 cosd(thetax)*thetax_dot -sind(thetax)*cosd(thetay)*thetax_dot-cosd(thetax)*sind(thetay)*thetay_dot];
Sx_dot=[zeros(3,3) zeros(3,3);zeros(3,3) Sx_dot_2_2];
% Sx_dot=zeros(6,6); %简化使得导数均为0

%% 通过cp得到c(x)=Rxyz*cp,进而得到c^(x)是c(x)对应的斜对称矩阵
cx=Rxyz*cp;
v1=cx(1);
v2=cx(2);
v3=cx(3);
c_sharp_x=[0 -v3 v2;v3 0 -v1;-v2 v1 0];

%% 得到g(x)
gx=mp*g*[0 0 -1 -cx(2) cx(1) 0]';

%% 得到H(x)=Rxyz*IG*Rxyz'+mp*c_sharp_x*c_sharp_x';
Hx=Rxyz*IG*Rxyz'+mp*c_sharp_x*c_sharp_x';

%% 得到M'(x)
M_apo_x=[mp*ones(3,3) -mp*c_sharp_x; mp*c_sharp_x Hx];

%% 得到C'(x)
C_apo_x=[mp*omiga_sharp*omiga_sharp*cx; omiga_sharp*Hx*omiga];

%% 最终得到索力分布t
f=(M_apo_x*(Sx_dot*x_dot+Sx*x_ddot)+C_apo_x-gx);
end

