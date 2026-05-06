function [feas] = PM_Method_Func(p,we,Rxyz)
%% 输入输出说明
% 输入参数：
% p:动平台原点的的坐标 (x,y,z)'
% we:施加在动平台上的wrench (f1,f2,f3,T1,T2,T3)'
% Rxyz:动平台的旋转矩阵
% 输出参数：
% feas:该位姿，该wrench下是否力可行，0代表不可行，1代表可行
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
for i=1:8
    expr=['l',num2str(i),'=a',num2str(i),'-b',num2str(i),'-p;'];
    eval(expr);
end
% si=li/norm(li)
for i=1:8
    expr=['s',num2str(i),'=l',num2str(i),'./norm(l',num2str(i),');'];
    eval(expr);
end

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

%% 求解W矩阵的逆矩阵Wplus和零空间矩阵N
Wplus=pinv(W);
N=null(W);
% 使用C++通过递归得到从W的8行里面选取5行的所有可能排序
Sort=[1 2 3 4 5 6 7 8];
PosibleSort=[
1       2       3       4       5
1       2       3       4       6
1       2       3       4       7
1       2       3       4       8
1       2       3       5       6
1       2       3       5       7
1       2       3       5       8
1       2       3       6       7
1       2       3       6       8
1       2       3       7       8
1       2       4       5       6
1       2       4       5       7
1       2       4       5       8
1       2       4       6       7
1       2       4       6       8
1       2       4       7       8
1       2       5       6       7
1       2       5       6       8
1       2       5       7       8
1       2       6       7       8
1       3       4       5       6
1       3       4       5       7
1       3       4       5       8
1       3       4       6       7
1       3       4       6       8
1       3       4       7       8
1       3       5       6       7
1       3       5       6       8
1       3       5       7       8
1       3       6       7       8
1       4       5       6       7
1       4       5       6       8
1       4       5       7       8
1       4       6       7       8
1       5       6       7       8
2       3       4       5       6
2       3       4       5       7
2       3       4       5       8
2       3       4       6       7
2       3       4       6       8
2       3       4       7       8
2       3       5       6       7
2       3       5       6       8
2       3       5       7       8
2       3       6       7       8
2       4       5       6       7
2       4       5       6       8
2       4       5       7       8
2       4       6       7       8
2       5       6       7       8
3       4       5       6       7
3       4       5       6       8
3       4       5       7       8
3       4       6       7       8
3       5       6       7       8
4       5       6       7       8];
%计算Wj+ Wj- Nj+ Nj- ，计算alpaj=null(Nj-'),计算cj=Wj-'*alpaj
%
for i=1:56
    %计算Wj+ Nj+
    for j=1:5
        row=PosibleSort(i,j);
        Wj_plus(j,:)=Wplus(row,:);
        Nj_plus(j,:)=N(row,:);
    end
    %计算Wj- Nj-
    inverse_sort=setdiff(Sort,PosibleSort(i,:));
    for k=1:3
        inverse_row=inverse_sort(k);
        Wj_sub(k,:)=Wplus(inverse_row,:);
        Nj_sub(k,:)=N(inverse_row,:);
    end
    alpaj=null(Nj_sub');
    if(size(alpaj,2)==0)
        feas=0;
        return;
    end
    alpaj=alpaj(:,1); %取零空间的其中一个向量
    cj=Wj_sub'*alpaj;
    % 计算dj+ dj-
    dj_plus=0;
    dj_sub=0;
    for l=1:3
        if alpaj(l,:)>0
            dj_plus=dj_plus+tM*alpaj(l,:);
            dj_sub=dj_sub-tm*alpaj(l,:);
        end
        if alpaj(l,:)<0
            dj_plus=dj_plus+tm*alpaj(l,:);
            dj_sub=dj_sub-tM*alpaj(l,:);
        end
    end
    cj_plus=cj;
    cj_sub=-cj;
    C(:,2*i-1)=cj_plus;
    C(:,2*i)=cj_sub;
    d(2*i-1,:)=dj_plus;
    d(2*i,:)=dj_sub;
end

feas=1;
ans=C'*we;
for i=1:112
    if ans(i)<=d(i)
    end
    if ans(i)>d(i)
       feas=0;
       return
    end
end
end