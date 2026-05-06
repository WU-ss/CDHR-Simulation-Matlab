function [q,q_dot] = SCDPR_Func(tao_in,t_in,q_1,q_dot_1)
% 主要求解电机的动力学方程，通过给定电机转矩以及索力信息，得到电机的角度以及角速度

% global tao;
tao=tao_in;
% global tt;
tt=t_in;
% global Im;
Im=[0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]';
% global Fv;
Fv=[1 1 1 1 1 1 1 1]';
% global Fs;
Fs=[1 1 1 1 1 1 1 1]';
% global R;
R=[0.0225 0.0225 0.0225 0.0225 0.0225 0.0225 0.0225 0.0225]';

T=0.001;

q=zeros(8,1);
q_dot=zeros(8,1);


% 第1个微分方程
Y11=q_1(1);
Y21=q_dot_1(1);
% 解微分方程
[t,q1] = ode45(@vdp1,[0 T],[Y11; Y21],[],tao,tt,Im,Fv,Fs,R);
q(1)=q1(end,1);
q_dot(1)=q1(end,2);

% 第2个微分方程
Y12=q_1(2);
Y22=q_dot_1(2);
% 解微分方程
[t,q2] = ode45(@vdp2,[0 T],[Y12; Y22],[],tao,tt,Im,Fv,Fs,R);
q(2)=q2(end,1);
q_dot(2)=q2(end,2);

% 第3个微分方程
Y13=q_1(3);
Y23=q_dot_1(3);
% 解微分方程
[t,q3] = ode45(@vdp3,[0 T],[Y13; Y23],[],tao,tt,Im,Fv,Fs,R);
q(3)=q3(end,1);
q_dot(3)=q3(end,2);

% 第4个微分方程
Y14=q_1(4);
Y24=q_dot_1(4);
% 解微分方程
[t,q4] = ode45(@vdp4,[0 T],[Y14; Y24],[],tao,tt,Im,Fv,Fs,R);
q(4)=q4(end,1);
q_dot(4)=q4(end,2);

% 第5个微分方程
Y15=q_1(5);
Y25=q_dot_1(5);
% 解微分方程
[t,q5] = ode45(@vdp5,[0 T],[Y15; Y25],[],tao,tt,Im,Fv,Fs,R);
q(5)=q5(end,1);
q_dot(5)=q5(end,2);

% 第6个微分方程
Y16=q_1(6);
Y26=q_dot_1(6);
% 解微分方程
[t,q6] = ode45(@vdp6,[0 T],[Y16; Y26],[],tao,tt,Im,Fv,Fs,R);
q(6)=q6(end,1);
q_dot(6)=q6(end,2);

% 第7个微分方程
Y17=q_1(7);
Y27=q_dot_1(7);
% 解微分方程
[t,q7] = ode45(@vdp7,[0 T],[Y17; Y27],[],tao,tt,Im,Fv,Fs,R);
q(7)=q7(end,1);
q_dot(7)=q7(end,2);

% 第8个微分方程
Y18=q_1(8);
Y28=q_dot_1(8);
% 解微分方程
[t,q8] = ode45(@vdp8,[0 T],[Y18; Y28],[],tao,tt,Im,Fv,Fs,R);
q(8)=q8(end,1);
q_dot(8)=q8(end,2);

function dydt1 = vdp1(t,y,tao,tt,Im,Fv,Fs,R)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
    
    tao1=tao(1);
    tt1=tt(1);
    Im1=Im(1);
    Fv1=Fv(1);
    Fs1=Fs(1);
    R1=R(1);

    p = y(1);
    dp = y(2);
% 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
    ddp = -Fv1/Im1*dp-Fs1/Im1*sign(dp)+tao1/Im1-R1/Im1*tt1;
    dydt1 = [dp; ddp];
end

function dydt2 = vdp2(t,y,tao,tt,Im,Fv,Fs,R)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
    
    tao2=tao(2);
    tt2=tt(2);
    Im2=Im(2);
    Fv2=Fv(2);
    Fs2=Fs(2);
    R2=R(2);

    p = y(1);
    dp = y(2);
% 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
    ddp = -Fv2/Im2*dp-Fs2/Im2*sign(dp)+tao2/Im2-R2/Im2*tt2;
    dydt2 = [dp; ddp];
end

function dydt3 = vdp3(t,y,tao,tt,Im,Fv,Fs,R)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
    
    tao3=tao(3);
    tt3=tt(3);
    Im3=Im(3);
    Fv3=Fv(3);
    Fs3=Fs(3);
    R3=R(3);

    p = y(1);
    dp = y(2);
% 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
    ddp = -Fv3/Im3*dp-Fs3/Im3*sign(dp)+tao3/Im3-R3/Im3*tt3;
    dydt3 = [dp; ddp];
end

function dydt4 = vdp4(t,y,tao,tt,Im,Fv,Fs,R)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
    
    tao4=tao(4);
    tt4=tt(4);
    Im4=Im(4);
    Fv4=Fv(4);
    Fs4=Fs(4);
    R4=R(4);

    p = y(1);
    dp = y(2);
% 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
    ddp = -Fv4/Im4*dp-Fs4/Im4*sign(dp)+tao4/Im4-R4/Im4*tt4;
    dydt4 = [dp; ddp];
end

function dydt5 = vdp5(t,y,tao,tt,Im,Fv,Fs,R)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
    
    tao5=tao(5);
    tt5=tt(5);
    Im5=Im(5);
    Fv5=Fv(5);
    Fs5=Fs(5);
    R5=R(5);

    p = y(1);
    dp = y(2);
% 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
    ddp = -Fv5/Im5*dp-Fs5/Im5*sign(dp)+tao5/Im5-R5/Im5*tt5;
    dydt5 = [dp; ddp];
end

function dydt6 = vdp6(t,y,tao,tt,Im,Fv,Fs,R)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
    
    tao6=tao(6);
    tt6=tt(6);
    Im6=Im(6);
    Fv6=Fv(6);
    Fs6=Fs(6);
    R6=R(6);

    p = y(1);
    dp = y(2);
% 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
    ddp = -Fv6/Im6*dp-Fs6/Im6*sign(dp)+tao6/Im6-R6/Im6*tt6;
    dydt6 = [dp; ddp];
end


function dydt7 = vdp7(t,y,tao,tt,Im,Fv,Fs,R)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
    
    tao7=tao(7);
    tt7=tt(7);
    Im7=Im(7);
    Fv7=Fv(7);
    Fs7=Fs(7);
    R7=R(7);

    p = y(1);
    dp = y(2);
% 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
    ddp = -Fv7/Im7*dp-Fs7/Im7*sign(dp)+tao7/Im7-R7/Im7*tt7;
    dydt7 = [dp; ddp];
end

function dydt8 = vdp8(t,y,tao,tt,Im,Fv,Fs,R)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
    
    tao8=tao(8);
    tt8=tt(8);
    Im8=Im(8);
    Fv8=Fv(8);
    Fs8=Fs(8);
    R8=R(8);

    p = y(1);
    dp = y(2);
% 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
    ddp = -Fv8/Im8*dp-Fs8/Im8*sign(dp)+tao8/Im8-R8/Im8*tt8;
    dydt8 = [dp; ddp];
end
end

