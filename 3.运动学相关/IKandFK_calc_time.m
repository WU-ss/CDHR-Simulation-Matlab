%% 本文件用于计算工作空间离散化后的IK与FK的耗时，以及FK的总误差
% CDORs框架的长宽高：15m*11m*6m
% 由于框架坐标系的原点位于底部中心位置，所以x:-7.5m-7.5m y:-5.5m-5.5m z=0m-6m,动平台本身为1m3的长方体
clc;
clear;
theta=[0 0 0]';
IKtime_vec=[];
FKtime_vec=[];
k_vec=[];
for x=-7:0.5:7 %防止动平台超出框架
    for y=-5:0.5:5
        for z=0.5:0.5:5
            xp=[x y z]';
            tic;
            [l,l_len] = InverseKenematics_Func(xp,theta);
            IKtime=toc
            l_len=l_len';
            tic;
            [solution,k,min_F] = ForwardKenamatics_Func(l_len);
            FKtime=toc
            IKtime_vec=[IKtime_vec,IKtime];
            FKtime_vec=[FKtime_vec,FKtime];
            k_vec=[k_vec,k];
        end
    end
end

FKtime_vec_round=round(FKtime_vec./(0.001)).*0.001.*1000; %单位是ms
FKtime_table=tabulate(FKtime_vec_round)

k_table=tabulate(k_vec) %单位是次数

IKtime_vec_round=round(IKtime_vec./(0.00001)).*0.00001.*1000; %单位是ms
IKtime_table=tabulate(IKtime_vec_round)

figure
subplot(1,3,1)
h1=histogram(FKtime_vec.*1000);
title('正运动学精确求解时间柱状图')
xlabel('时间/ms')
ylabel('出现次数')
xlim([0,6]);
subplot(1,3,2)
b1=bar(FKtime_table(:,1),FKtime_table(:,3)./100);
title('正运动学粗略求解时间概率分布柱状图')
xlabel('时间/ms')
ylabel('出现频率')
xlim([0,6]);
subplot(1,3,3)
hk=histogram(k_vec);
title('正运动学迭代次数柱状图')
xlabel('迭代次数k')
ylabel('出现次数')

figure
subplot(1,2,1)
h2=histogram(IKtime_vec.*1000);
title('逆运动学精确求解时间柱状图')
xlabel('时间/ms')
ylabel('出现次数')
xlim([0,0.1]);
subplot(1,2,2)
b2=bar(IKtime_table(:,1),IKtime_table(:,3)./100);
title('逆运动学粗略求解时间概率分布柱状图')
xlabel('时间/ms')
ylabel('出现频率')
xlim([0,0.1]);



