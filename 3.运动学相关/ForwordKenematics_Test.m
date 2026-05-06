%% 本文件用于测试CDPRs的正运动学，以便包装成函数
% 旋转采用XYZ欧拉角度描述
clc;
clear;
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
%% 给定li
l1=[-8 -5 3]';
l2=[-7 -6 2]';
l3=[-7 6 3]';
l4=[-8 5 2]';
l5=[8 5 3]';
l6=[7 6 2]';
l7=[7 -6 3]';
l8=[8 -5 2]';
L=[l1 l2 l3 l4 l5 l6 l7 l8];
l1_len=norm(l1);
l2_len=norm(l2);
l3_len=norm(l3);
l4_len=norm(l4);
l5_len=norm(l5);
l6_len=norm(l6);
l7_len=norm(l7);
l8_len=norm(l8);
s1=l1/l1_len;
s2=l2/l2_len;
s3=l3/l3_len;
s4=l4/l4_len;
s5=l5/l5_len;
s6=l6/l6_len;
s7=l7/l7_len;
s8=l8/l8_len;
%% 区间技术得到初始位置
r1_low=a1-(l1_len-norm(b1))*[1 1 1]';
r1_high=a1+(l1_len-norm(b1))*[1 1 1]';
r2_low=a2-(l2_len-norm(b2))*[1 1 1]';
r2_high=a2+(l2_len-norm(b2))*[1 1 1]';
r3_low=a3-(l3_len-norm(b3))*[1 1 1]';
r3_high=a3+(l3_len-norm(b3))*[1 1 1]';
r4_low=a4-(l4_len-norm(b4))*[1 1 1]';
r4_high=a4+(l4_len-norm(b4))*[1 1 1]';
r5_low=a5-(l5_len-norm(b5))*[1 1 1]';
r5_high=a5+(l5_len-norm(b5))*[1 1 1]';
r6_low=a6-(l6_len-norm(b6))*[1 1 1]';
r6_high=a6+(l6_len-norm(b6))*[1 1 1]';
r7_low=a7-(l7_len-norm(b7))*[1 1 1]';
r7_high=a7+(l7_len-norm(b7))*[1 1 1]';
r8_low=a8-(l8_len-norm(b8))*[1 1 1]';
r8_high=a8+(l8_len-norm(b8))*[1 1 1]';

r_low_arry=[r1_low r2_low r3_low r4_low r5_low r6_low r7_low r8_low];
r_low_sqrt=[norm(r1_low) norm(r2_low) norm(r3_low) norm(r4_low) norm(r5_low) norm(r6_low) norm(r7_low) norm(r8_low)];
[r_low_value,r_low_pos]=max(r_low_sqrt);
r_low=r_low_arry(:,r_low_pos);

r_high_arry=[r1_high r2_high r3_high r4_high r5_high r6_high r7_high r8_high];
r_high_sqrt=[norm(r1_high) norm(r2_high) norm(r3_high) norm(r4_high) norm(r5_high) norm(r6_high) norm(r7_high) norm(r8_high)];
[r_high_value,r_high_pos]=max(r_high_sqrt);
r_high=r_high_arry(:,r_high_pos);

% r_init=1/2*(r_low+r_high);
r_init=[0 2 2]';
theta_init=[0 0 0]';
%% 使用LM算法迭代求解
sigma1=1e-10; %规定迭代的结束条件使用的无穷小数
sigma2=1e-10; %规定迭代的结束条件使用的无穷小数
tao=1e-5;
k=0;
v=2;
Y_init=[r_init;theta_init];
Y=Y_init;
[finit,Jinit] = LM_Update_f_and_J_Func(L,Y_init);
f=finit;
H=Jinit'*Jinit;
g=Jinit'*finit;
miu=tao*max(max(H));
found=(max(g)<sigma1);
while found==0
    k=k+1;
    hlm=-inv(H+miu*eye(6))*g;
    if norm(hlm)<sigma2*(norm(Y)+sigma2)
        found=1;
    else
        Ynew=Y+hlm;
        [fnew,Jnew] = LM_Update_f_and_J_Func(L,Ynew);
        F=1/2*(f')*f;
        Fnew=1/2*(fnew')*fnew;
        L_zero=1/2*(f')*f;
        Lhlm=1/2*(f')*(f)+(hlm')*(Jnew')*(f)+1/2*(hlm')*(Jnew')*(Jnew)*(hlm)+1/2*miu*(hlm')*(hlm);
%         rou=(F-Fnew)/(L_zero-Lhlm);
        rou=(F-Fnew)/(1/2*hlm'*(miu*hlm-g));
        if rou>0
            Y=Ynew;
            f=fnew;
            H=Jnew'*Jnew;
            g=Jnew'*fnew;
            found=(max(g)<=sigma1);
            miu=miu*max(1/3,(1-(2*rou-1)^3));
            v=2;
        else
            miu=miu*v;
            v=2*v;
        end
    end    
end


