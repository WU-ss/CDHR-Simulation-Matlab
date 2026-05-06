%% 主要计算CDPRs的索力分布，采用论文2的方法，方便进行图像的绘制
clc;
clear;

mp=50; %动平台的质量
g=9.81; %重力加速度

p=[0 0 5]';
theta=[60 -60 60]'; %动平台的夹角 XYZ欧拉角

x=theta(1);
y=theta(2);
z=theta(3);
Rxyz=Theta2RotationMatrix_Func(x,y,z);
xGp=[0 -0.075 0]'; %动平台COM相对于动平台的原点的位置偏移
% xGp=[0 0 0]'; %动平台COM相对于动平台的原点的位置偏移
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
l1=a1-b1-p;
l2=a2-b2-p;
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

W_plus=pinv(W);
N=null(W);
tp=W_plus*f;

tm_vec=[tm;tm;tm;tm;tm;tm;tm;tm;];
tM_vec=[tM;tM;tM;tM;tM;tM;tM;tM;];

% line_select=[];
% langda_select=[];

find=0;
k_vector_all=[1;2;3;4;5;6;7;8];

%% 寻找第一个顶点vinit
i=1;
j=2;
bi=tm;
bj=tm;
bk_1=0;
bk_2=0;
ni=N(i,:);
nj=N(j,:);
tpi=tp(i,:);
tpj=tp(j,:);
A=[ni;nj];
b=[bi-tpi;bj-tpj];
langda=A\b; %2*1向量
vij=langda;

% line_select=[line_select;j];
% line_select=[line_select;i];
line_select=[i];

lk_1=i;
lk_2=j;

% line_find=[];
% line_find=[line_find;1];
% line_find=[line_find;2];
line_find=[i;j];

for i=3:8
    ni_2=N(i,:);
    tpi_2=tp(i,:);
    if ni_2*langda>=tm-tpi_2 & ni_2*langda<=tM-tpi_2
        line_find=[line_find;i];
    end
    
end


line_find
langda_select=[vij];


k_vector=setdiff(k_vector_all,line_select);

%% 循环找下一个节点，直到找到vk==vf
vk=[1e30;1e30];
% alpah_v=[1e30;1e30];
alpah_v=[1e30];
vk_1=vij;
nk_1=ni;
nk_2=nj;
nk_1_vertical=null(nk_1)';
nk_2_vertical=null(nk_2)';
l=0;
bk=0;
tpk=0;
count=0;
find2=0;
vf=vij;
vf_count=count+1;
nk_1_vertical_final=[0 0];
while(1)
%     alpah_k=[1e30;1e30];
    alpah_k=[1e30];
    count=count+1;
    find2=0;
    nk_1_vertical_final=[0 0];
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
                if k==lk_2
                    if nk*vk_1<=tm-tpk
                        alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=1;
                    elseif nk*vk_1>tm-tpk & nk*vk_1<=tM-tpk
                        alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=1;
                    else
                        bk_tmp=0;
                    end
%                     if nk*vk_1>tm-tpk & nk*vk_1<=tM-tpk
%                         alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
%                         bk_tmp=1;
%                     end
                else
                    if nk*vk_1<=tm-tpk
                        alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=0;
                    elseif nk*vk_1>tm-tpk & nk*vk_1<=tM-tpk
                        alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=1;
                    else
                        bk_tmp=0;
                    end
%                     if nk*vk_1>tm-tpk & nk*vk_1<=tM-tpk
%                         alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
%                         bk_tmp=1;
%                     end
                end
                if (alpah_k<alpah_v) & alpah_k>=1e-6
                    alpah_v=alpah_k;
                    l=k;
                    find2=1;
                    nk_1_vertical_final=nk_1_vertical;
                    bk=bk_tmp;
                else
                    
                end
            end

            if nk*nk_1_vertical'<0
                if k==lk_2
                    if nk*vk_1>=tM-tpk
                        alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=0;
                    elseif nk*vk_1>=tm-tpk & nk*vk_1<tM-tpk
                        alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=0;
                    else
                        bk_tmp=0;
                    end
%                     if nk*vk_1>=tm-tpk & nk*vk_1<tM-tpk
%                         alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
%                         bk_tmp=0;
%                     end
                else
                    if nk*vk_1>=tM-tpk
                        alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=1;
                    elseif nk*vk_1>=tm-tpk & nk*vk_1<tM-tpk
                        alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=0;
                    else
                        bk_tmp=0;
                    end
%                     if nk*vk_1>=tm-tpk & nk*vk_1<tM-tpk
%                         alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
%                         bk_tmp=0;
%                     end
                end
                if (alpah_k<alpah_v) & alpah_k>=1e-6
                    alpah_v=alpah_k;
                    l=k;
                    find2=1;
                    nk_1_vertical_final=nk_1_vertical;
                    bk=bk_tmp;
                else
                    
                end
            end
        
        % 考虑bk=tM的情况
        elseif bk_2==1
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
                if k==lk_2
                    if nk*vk_1<=tm-tpk
                        alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=1;
                    elseif nk*vk_1>tm-tpk & nk*vk_1<=tM-tpk
                        alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=1;
                    else
                        bk_tmp=0;
                    end
%                     if nk*vk_1>tm-tpk & nk*vk_1<=tM-tpk
%                         alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
%                         bk_tmp=1;
%                     end
                else
                    if nk*vk_1<=tm-tpk
                        alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=0;
                    elseif nk*vk_1>tm-tpk & nk*vk_1<=tM-tpk
                        alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=1;
                    else
                        bk_tmp=0;
                    end
%                     if nk*vk_1>tm-tpk & nk*vk_1<=tM-tpk
%                         alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
%                         bk_tmp=1;
%                     end
                end
                if (alpah_k<alpah_v) & alpah_k>=1e-6
                    alpah_v=alpah_k;
                    l=k;
                    find2=1;
                    nk_1_vertical_final=nk_1_vertical;
                    bk=bk_tmp;
                else
                    
                end
            end

            if nk*nk_1_vertical'<0
                if k==lk_2
                    if nk*vk_1>=tM-tpk
                        alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=0;
                    elseif nk*vk_1>=tm-tpk & nk*vk_1<tM-tpk
                        alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=0;
                    else
                        bk_tmp=0;
                    end
%                     if nk*vk_1>=tm-tpk & nk*vk_1<tM-tpk
%                         alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
%                         bk_tmp=0;
%                     end
                else
                    if nk*vk_1>=tM-tpk
                        alpah_k=(tM-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=1;
                    elseif nk*vk_1>=tm-tpk & nk*vk_1<tM-tpk
                        alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
                        bk_tmp=0;
                    else
                        bk_tmp=0;
                    end
%                     if nk*vk_1>=tm-tpk & nk*vk_1<tM-tpk
%                         alpah_k=(tm-tpk-nk*vk_1)/(nk*nk_1_vertical');
%                         bk_tmp=0;
%                     end
                end
                if (alpah_k<alpah_v) & alpah_k>=1e-6
                    alpah_v=alpah_k;
                    l=k;
                    find2=1;
                    nk_1_vertical_final=nk_1_vertical;
                    bk=bk_tmp;
                else
                    
                end
            end
        else
            find2=0;
        end
    end
    if find2==0 || count>1000
        t=[0 0 0 0 0 0 0 0]';
        bool=0;
        disp("索力分布无解");
        break
    else
        vk=vk_1+alpah_v.*nk_1_vertical_final';
        nk=N(l,:);
        langda_select=[langda_select,vk];
    %     line_select(1,:)=line_select(2,:);
    %     line_select(2,:)=l;
        line_select(1)=l;
        lk=l;
        k_vector=setdiff(k_vector_all,line_select);
        vk_2=vk_1;
        vk_1=vk;
        nk_2=nk_1;
        nk_1=nk;
        bk_2=bk_1;
        bk_1=bk;
        lk_2=lk_1;
        lk_1=lk;
    %     alpah_v=[1e10;1e10];
        alpah_v=[1e30];
        l
        if ~ismember(l,line_find)
            line_find=[line_find;l];
            line_find=sort(line_find);
            vf=vk;
            vf_count=count+1;
        else
            if abs(vf(1)-vk(1))<1e-3 & abs(vf(2)-vk(2))<1e-3
                line_find=sort(line_find);
                if isequal(line_find,k_vector_all)
                    disp('查找成功');
                    bool=1;
                    break;
                else
                    t=[0 0 0 0 0 0 0 0]';
                    bool=0;
                    disp('查找失败');
                    return
                end
            else
                line_find=line_find;
            end
        end        
    end
    
end

langda_final_vce=langda_select(1:2,end-(size(langda_select,2)-vf_count):end-1);

%% 重心法得到多边形质心
% 求解A
A=0;
vc1=0;
vc2=0;

count=size(langda_select,2)-vf_count;

if count==3
    vc1=(langda_final_vce(1,1)+langda_final_vce(1,2)+langda_final_vce(1,3))/3;
    vc2=(langda_final_vce(2,1)+langda_final_vce(2,2)+langda_final_vce(2,3))/3;
end

if count==4
    vc1=(langda_final_vce(1,1)+langda_final_vce(1,2)+langda_final_vce(1,3)+langda_final_vce(1,4))/4;
    vc2=(langda_final_vce(2,1)+langda_final_vce(2,2)+langda_final_vce(2,3)+langda_final_vce(2,4))/4;
end

if count>4
    for p=1:size(langda_final_vce,2)-1
        A=A+(langda_final_vce(1,p)*langda_final_vce(2,p+1)-langda_final_vce(1,p+1)*langda_final_vce(2,p))/2;
    end

    for p=1:size(langda_final_vce,2)-1
        vc1=vc1+((langda_final_vce(1,p)+langda_final_vce(1,p+1))*(langda_final_vce(1,p)*langda_final_vce(2,p+1)-langda_final_vce(1,p+1)*langda_final_vce(2,p)))/(6*A);
        vc2=vc2+((langda_final_vce(2,p)+langda_final_vce(2,p+1))*(langda_final_vce(1,p)*langda_final_vce(2,p+1)-langda_final_vce(1,p+1)*langda_final_vce(2,p)))/(6*A);
    end
end

% vc1=456.6389;
% vc2=534.3178;

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
    if i==1
        h1_m=plot(langda1,langda2,'r-');
    else
        plot(langda1,langda2,'r-');
    end
    hold on
end

for i=1:8 
    ni=N(i,:);
    tpi=tp(i,:);
    ni1=ni(:,1);
    ni2=ni(:,2);
    langda1=-4000:0.1:4000;
    langda2=(tM-tpi-ni1*langda1)/ni2;
    if i==1
        h1_M=plot(langda1,langda2,'b-');
    else
        plot(langda1,langda2,'b-');
    end
    hold on
end

for i=1:size(langda_select,2)
    if i==1
        h3=plot(langda_select(1,i),langda_select(2,i),'m.','MarkerSize',10);
    else
        plot(langda_select(1,i),langda_select(2,i),'m.','MarkerSize',10);
    end    
end

for i=1:size(langda_select,2)-1
    text(langda_select(1,i)+30,langda_select(2,i)+30,num2str(i)) ; %加上0.01使标号和点不重合，可以调整
end



for i=1:size(langda_final_vce,2)
    if i==1
        h4=plot(langda_final_vce(1,i),langda_final_vce(2,i),'b*','MarkerSize',5); 
    else
        plot(langda_final_vce(1,i),langda_final_vce(2,i),'b*','MarkerSize',5) ;
    end         
end


h5=plot(vc1,vc2,'r.','MarkerSize',10);
text(vc1-250,vc2-100,'Centroid','FontSize',11,'Color','red') ; %加上0.01使标号和点不重合，可以调整

legend([h1_m,h1_M,h3,h4,h5],'索力下限tm形成的直线','索力上限tM形成的直线','未满足所有不等式的点','已满足所有不等式的点','多边形的质心');
xlabel('λ1')
ylabel('λ2') 
xlim([-2000 2000])
ylim([-2000 2000])

