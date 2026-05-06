function [solution,k,minF] = ForwardKenamatics_Func(L)
%*********************************************************
%使用LM法求解运动学正解，迭代起点在base内部空间任意选择；
%输入参数：L 绳索长度构成的行向量；
%输出参数：solution 动平台的位姿向量
%         k 迭代次数
%         minF 误差F的最终值
%*********************************************************
%%
%base/plateform的基本数据：
%base:L*W*H=15*11*6
%drawing points在O-XYZ中表示：
A(1:3,1)=[-7.5 -5.5 6]';
A(1:3,2)=[-7.5 -5.5 6];
A(1:3,3)=[-7.5 5.5 6]';
A(1:3,4)=[-7.5 5.5 6]';
A(1:3,5)=[7.5 5.5 6]';
A(1:3,6)=[7.5 5.5 6];
A(1:3,7)=[7.5 -5.5 6];
A(1:3,8)=[7.5 -5.5 6]';
%plateform:l*w*h=1*1*1
%attach points在P-XpYpZp中表示：
Bp(1:3,1)=[0.5 -0.5 0]';
Bp(1:3,2)=[-0.5 0.5 1]';
Bp(1:3,3)=[-0.5 -0.5 0]';
Bp(1:3,4)=[0.5 0.5 1]';
Bp(1:3,5)=[-0.5 0.5 0]';
Bp(1:3,6)=[0.5 -0.5 1]';
Bp(1:3,7)=[0.5 0.5 0]';
Bp(1:3,8)=[-0.5 -0.5 1]';

%%
%约束函数向量为vector_f；目标函数为F;
%采用《methods for non-linear least squares problems》中表述：F=1/2*sum(fi(x)^2)=1/2*(vector_f'*vector_f)
length_given=L;
%下面为LM法参数：
%设置stop criteria:
epsilon1=1.0e-8;
epsilon2=1.0e-8;
%设置迭代次数上限：
k_max=100;
%当前迭代次数：
k=0;
%设置初始迭代位置:设置为固定平台中心位置
%pose_position_angle=[x y z angle_x angle_y angle_z]
pose_position_angle=[0 0 1 0 0 0];
%u的增长因子v
v=2;
%结束标志
found=false;

%% 统计时间的开始
%LM算法计算开始：
jacobian_f=Jaco(pose_position_angle,A,Bp);
A_matrix=jacobian_f'*jacobian_f;
vector_f=FunctionVector_f(pose_position_angle,A,Bp,length_given);
g=jacobian_f'*vector_f;

if norm(g)<=epsilon1
    found=true;
end

%设置初始damped parameter u
tau=1.0e-3;
u=tau*max([A_matrix(1,1),A_matrix(2,2),A_matrix(3,3),A_matrix(4,4),A_matrix(5,5),A_matrix(6,6)]);

while(found==false)&&(k<=k_max)
    k=k+1;
    %解的迭代方向及步长
    h_lm=(A_matrix+u*eye(6,6))\(-1*g);
    %h_lm=inv(A_matrix+u*eye(6,6))*(-1*g);
    if norm(h_lm)<=(epsilon2*(norm(pose_position_angle))+epsilon2)
        found=true;
    else
        pose_position_angle_new=pose_position_angle+h_lm';
        vector_f_new=FunctionVector_f(pose_position_angle_new,A,Bp,length_given);
        %计算更新指标Q
        Q=Q_criteria(vector_f,vector_f_new,h_lm,u,g);
        
        if Q>0
            
            pose_position_angle=pose_position_angle_new;
            
            jacobian_f=Jaco(pose_position_angle,A,Bp);
            A_matrix=jacobian_f'*jacobian_f;
            vector_f=FunctionVector_f(pose_position_angle,A,Bp,length_given);
            g=jacobian_f'*vector_f;
            
            if norm(g)<=epsilon1
                found=true;
            end
            u=u*max([1/3, 1, 1-(2*Q-1)^3]);
            v=2;
        else
            u=u*v;
            v=2*v;
        end
        
    end
    
    
end

solution=pose_position_angle';
vector_f=FunctionVector_f(pose_position_angle,A,Bp,length_given);
minF=1/2*vector_f'*vector_f;
end

