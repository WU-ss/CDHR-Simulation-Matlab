function [p_dot,theta_dot] = Inverse_Jacobian_Func(q_dot,p,theta)
% 雅可比矩阵，从位姿x到绳长l，再到电机角度q

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
b1=Rxyz*[0.5 -0.5 0]';
b2=Rxyz*[-0.5 0.5 1]';
b3=Rxyz*[-0.5 -0.5 0]';
b4=Rxyz*[0.5 0.5 1]';
b5=Rxyz*[-0.5 0.5 0]';
b6=Rxyz*[0.5 -0.5 1]';
b7=Rxyz*[0.5 0.5 0]';
b8=Rxyz*[-0.5 -0.5 1]';
% 绳索张力的上下限
tM=1000;
tm=10;
%% li=ai-bi-p,然后归一化li得到si=li/norm(li)
% li=ai-bi-p
l1=a1-b1-p;
l2=a2-b3-p;
l3=a3-b3-p;
l4=a4-b4-p;
l5=a5-b5-p;
l6=a6-b6-p;
l7=a7-b7-p;
l8=a8-b8-p;
% for i=1:8
%     expr=['l',num2str(i),'=a',num2str(i),'-b',num2str(i),'-p;'];
%     eval(expr);
% end
% si=li/norm(li)
s1=l1/norm(l1);
s2=l2/norm(l2);
s3=l3/norm(l3);
s4=l4/norm(l4);
s5=l5/norm(l5);
s6=l6/norm(l6);
s7=l7/norm(l7);
s8=l8/norm(l8);
% for i=1:8
%     expr=['s',num2str(i),'=l',num2str(i),'./norm(l',num2str(i),');'];
%     eval(expr);
% end

%% 构建W矩阵
% wi=[si;cross(bi,si)]
w1=[s1;cross(b1,s1)];
w2=[s2;cross(b2,s2)];
w3=[s3;cross(b3,s3)];
w4=[s4;cross(b4,s4)];
w5=[s5;cross(b5,s5)];
w6=[s6;cross(b6,s6)];
w7=[s7;cross(b7,s7)];
w8=[s8;cross(b8,s8)];
W=[w1 w2 w3 w4 w5 w6 w7 w8];

%% 得到雅可比矩阵
J=-W';
J_plus=pinv(J);
l_dot=q_dot*0.0225;
x_dot=J_plus*l_dot;
p_dot=x_dot(1:3,:);
theta_dot=x_dot(4:6,:);

end

