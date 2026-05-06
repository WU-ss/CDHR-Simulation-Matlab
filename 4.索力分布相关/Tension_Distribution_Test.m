%% 主要计算CDPRs的索力分布，方便封装成函数
clc;
clear;

mp=79; %动平台的质量
g=9.81; %重力加速度

p=[-3 -4 3]';
theta=[0 0 0]'; %动平台的夹角 XYZ欧拉角

x=theta(1);
y=theta(2);
z=theta(3);
Rxyz=Theta2RotationMatrix_Func(x,y,z);
% xGp=[0 0 0]'; %动平台COM相对于动平台的原点的位置偏移
xGp=[0 -0.075 0]'; %动平台COM相对于动平台的原点的位置偏移
xGp=Rxyz*xGp; %考虑旋转的作用
zb=[0 0 1]'; %框架坐标系下z轴的单位向量
f=-mp*g*zb;
T=-mp*g*cross(xGp,zb);
wp=-[f;T];
f=wp;

% 求解索力分布的函数
% 输入：p 动平台位置 3*1
%      Rxyz 动平台坐标系旋转矩阵 3*3
%      f 动平台受到的wrench 6*1
% 输出：t 绳索的张力的大小 8*1
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

W_plus=pinv(W);
N=null(W);
tp=W_plus*f;

bi=tm;
bj=tm;

tm_vec=[tm;tm;tm;tm;tm;tm;tm;tm;];
tM_vec=[tM;tM;tM;tM;tM;tM;tM;tM;];

line_select=[];
langda_select=[];

find=0;
k_vector_all=[1;2;3;4;5;6;7;8];

%% 寻找第一个顶点vinit
while(1)
    bi=tm;
    bj=tm;
    for i=1:8
        if find
            break;
        end
        for j=i+1:8
            ni=N(i,:);
            nj=N(j,:);
            tpi=tp(i,:);
            tpj=tp(j,:);
            A=[ni;nj];
            b=[bi-tpi;bj-tpj];
            langda=A\b; %2*1向量
            x=N*langda;
            tmm=tm_vec-tp;
            tMM=tM_vec-tp;
            if (Cmp_Vector_Func(x,tmm,0)) && (Cmp_Vector_Func(x,tMM,1))
                line_select=[line_select;j];
                line_select=[line_select;i];
%                 line_select=[i];
                langda_select=[langda_select;langda];
                vinit=langda;
                find=1;
                ni_final=ni;
                nj_final=nj;
                bj_final=bj;
                tpj_final=tpj;
                bk_2=0;
                bk_1=0;
                break;
            end
        end
    end
    
    if  find==1
        break
    end  
    
    bi=tm;
    bj=tM;
    for i=1:8
        if find
            break;
        end
        for j=i+1:8
            ni=N(i,:);
            nj=N(j,:);
            tpi=tp(i,:);
            tpj=tp(j,:);
            A=[ni;nj];
            b=[bi-tpi;bj-tpj];
            langda=A\b; %2*1向量
            x=N*langda;
            tmm=tm_vec-tp;
            tMM=tM_vec-tp;
            if (Cmp_Vector_Func(x,tmm,0)) && (Cmp_Vector_Func(x,tMM,1))
                line_select=[line_select;j];
                line_select=[line_select;i];
    %             line_select=[i];
                langda_select=[langda_select;langda];
                vinit=langda;
                find=1;
                ni_final=ni;
                nj_final=nj;
                bj_final=bj;
                tpj_final=tpj;
                bk_2=1;
                bk_1=0;
                break;
            end
        end
    end

    if  find==1
        break
    end
    
    bi=tM;
    bj=tm;
    for i=1:8
        if find
            break;
        end
        for j=i+1:8
            ni=N(i,:);
            nj=N(j,:);
            tpi=tp(i,:);
            tpj=tp(j,:);
            A=[ni;nj];
            b=[bi-tpi;bj-tpj];
            langda=A\b; %2*1向量
            x=N*langda;
            tmm=tm_vec-tp;
            tMM=tM_vec-tp;
            if (Cmp_Vector_Func(x,tmm,0)) && (Cmp_Vector_Func(x,tMM,1))
                line_select=[line_select;j];
                line_select=[line_select;i];
    %             line_select=[i];
                langda_select=[langda_select;langda];
                vinit=langda;
                find=1;
                ni_final=ni;
                nj_final=nj;
                bj_final=bj;
                tpj_final=tpj;
                bk_2=0;
                bk_1=1;
                break;
            end
        end
    end

    if  find==1
        break
    end
    bi=tM;
    bj=tM;
    for i=1:8
        if find
            break;
        end
        for j=i+1:8
            ni=N(i,:);
            nj=N(j,:);
            tpi=tp(i,:);
            tpj=tp(j,:);
            A=[ni;nj];
            b=[bi-tpi;bj-tpj];
            langda=A\b; %2*1向量
            x=N*langda;
            tmm=tm_vec-tp;
            tMM=tM_vec-tp;
            if (Cmp_Vector_Func(x,tmm,0)) && (Cmp_Vector_Func(x,tMM,1))
                line_select=[line_select;j];
                line_select=[line_select;i];
    %             line_select=[i];
                langda_select=[langda_select;langda];
                vinit=langda;
                find=1;
                ni_final=ni;
                nj_final=nj;
                bj_final=bj;
                tpj_final=tpj;
                bk_2=1;
                bk_1=1;
                break;
            end
        end
    end
    
    if  find==1
        break
    end
    
    if(find==0)
        disp("索力分布无解")
        break
    end
end

k_vector=setdiff(k_vector_all,line_select);

%bk_2 bk_1 bk 0代表tm 1代表tM
%% 循环找下一个节点，直到找到vk==vinit
vk=[1e30;1e30];
alpah_v=[1e30;1e30];
vk_1=vinit;
nk_1=ni_final;
nk_2=nj_final;
nk_1_vertical=null(nk_1)';
nk_2_vertical=null(nk_2)';
l=0;
bk=0;
tpk=0;
count=0;
find2=0;
while( ~( (abs(nj_final*vk-(bj_final-tpj_final))<1e-3))  )
    alpah_k=[1e30;1e30];
    count=count+1;
    find2=0;
    for i=1:length(k_vector)
        k=k_vector(i);
        nk=N(k,:);
        tpk=tp(k,:);
        
        % 考虑bj=tm情况
        if bk_2==0           
            nk_1_vertical_1=[nk_1(2) -nk_1(1)];
            nk_1_vertical_2=[-nk_1(2) nk_1(1)];
%             nk_1_vertical_1=[-nk_1(2) nk_1(1)];
%             nk_1_vertical_2=[nk_1(2) -nk_1(1)];
            if nk_2*nk_1_vertical_1'>=0
                nk_1_vertical=nk_1_vertical_1;
            else
                nk_1_vertical=nk_1_vertical_2;
            end

            if nk*nk_1_vertical'>0
                if nk*vk_1<=tm-tpk
                    alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
                    bk_tmp=0;
                end
                if nk*vk_1>tm-tpk & nk*vk_1<=tM-tpk
                    alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
                    bk_tmp=1;
                end
                if (alpah_k<alpah_v) & alpah_k>=0
                    alpah_v=alpah_k;
                    l=k;
                    find2=1;
                    nk_1_vertical_final=nk_1_vertical;
                    bk=bk_tmp;
                end
            end

            if nk*nk_1_vertical'<0
                if nk*vk_1>=tM-tpk
                    alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
                    bk_tmp=1;
                end
                if nk*vk_1>=tm-tpk & nk*vk_1<tM-tpk
                    alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
                    bk_tmp=0;
                end
                if (alpah_k<alpah_v) & alpah_k>=0
                    alpah_v=alpah_k;
                    l=k;
                    find2=1;
                    nk_1_vertical_final=nk_1_vertical;
                    bk=bk_tmp;
                end
            end
        end
        
        % 考虑bk=tM的情况
        if bk_2==1
            nk_1_vertical_1=[nk_1(2) -nk_1(1)];
            nk_1_vertical_2=[-nk_1(2) nk_1(1)];
%             nk_1_vertical_1=[-nk_1(2) nk_1(1)];
%             nk_1_vertical_2=[nk_1(2) -nk_1(1)];
            if nk_2*nk_1_vertical_1'<=0
                nk_1_vertical=nk_1_vertical_1;
            else
                nk_1_vertical=nk_1_vertical_2;
            end

            if nk*nk_1_vertical'>0
                if nk*vk_1<=tm-tpk
                    alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
                    bk_tmp=0;
                end
                if nk*vk_1>tm-tpk & nk*vk_1<=tM-tpk
                    alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
                    bk_tmp=1;
                end
                if (alpah_k<alpah_v) & alpah_k>=0
                    alpah_v=alpah_k;
                    l=k;
                    find2=1;
                    nk_1_vertical_final=nk_1_vertical;
                    bk=bk_tmp;
                end
            end

            if nk*nk_1_vertical'<0
                if nk*vk_1>=tM-tpk
                    alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
                    bk_tmp=1;
                end
                if nk*vk_1>=tm-tpk & nk*vk_1<tM-tpk
                    alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
                    bk_tmp=0;
                end
                if (alpah_k<alpah_v) & alpah_k>=0
                    alpah_v=alpah_k;
                    l=k;
                    find2=1;
                    nk_1_vertical_final=nk_1_vertical;
                    bk=bk_tmp;
                end
            end
        end
    end
    if find2==0 || count>1000
        disp("索力分布无解");
        break
    end
    vk=vk_1+alpah_v.*nk_1_vertical_final';
    nk=N(l,:);
    langda_select=[langda_select,vk];
    line_select(1,:)=line_select(2,:);
    line_select(2,:)=l;
%     line_select(1)=l;
    k_vector=setdiff(k_vector_all,line_select);
    vk_2=vk_1;
    vk_1=vk;
    nk_2=nk_1;
    nk_1=nk;
    bk_2=bk_1;
    bk_1=bk;
    alpah_v=[1e10;1e10];
end

%% 重心法得到多边形质心
% 求解A
A=0;
vc1=0;
vc2=0;

if count==2
    vc1=(langda_select(1,1)+langda_select(1,2)+langda_select(1,3))/3;
    vc2=(langda_select(2,1)+langda_select(2,2)+langda_select(2,3))/3;
end

if count>2
    for p=1:size(langda_select,2)-1
        A=A+(langda_select(1,p)*langda_select(2,p+1)-langda_select(1,p+1)*langda_select(2,p))/2;
    end

    for p=1:size(langda_select,2)-1
        vc1=vc1+((langda_select(1,p)+langda_select(1,p+1))*(langda_select(1,p)*langda_select(2,p+1)-langda_select(1,p+1)*langda_select(2,p)))/(6*A);
        vc2=vc2+((langda_select(2,p)+langda_select(2,p+1))*(langda_select(1,p)*langda_select(2,p+1)-langda_select(1,p+1)*langda_select(2,p)))/(6*A);
    end
end

% vc1=161.5791;
% vc2=-18.4738;

langda_final=[vc1 vc2]';
t=tp+N*langda_final

%% 将该多边形画出来
figure;
for i=1:8 
    ni=N(i,:);
    tpi=tp(i,:);
    ni1=ni(:,1);
    ni2=ni(:,2);
    langda1=-4000:0.1:4000;
    langda2=(tm-tpi-ni1*langda1)/ni2;
    plot(langda1,langda2,'r-');
    hold on
end
for i=1:8 
    ni=N(i,:);
    tpi=tp(i,:);
    ni1=ni(:,1);
    ni2=ni(:,2);
    langda1=-4000:0.1:4000;
    langda2=(tM-tpi-ni1*langda1)/ni2;
    plot(langda1,langda2,'b-');
    hold on
end
for i=1:size(langda_select,2)
    plot(langda_select(1,i),langda_select(2,i),'m.','MarkerSize',10)
end
xlim([-4000 4000])
ylim([-4000 4000])

plot(vc1,vc2,'r.','MarkerSize',10)
