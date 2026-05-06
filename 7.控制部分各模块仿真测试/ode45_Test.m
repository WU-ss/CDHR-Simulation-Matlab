clc;
clear;

% global tao;
tao=[15 20 30 40 50 60 70 80]';
% global tt;
tt=[200 100 100 100 100 100 100 100]';
% global Im;
Im=[0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]';
% global Fv;
Fv=[1 1 1 1 1 1 1 1]';
% global Fs;
Fs=[1 1 1 1 1 1 1 1]';
% global R;
R=[0.0225 0.0225 0.0225 0.0225 0.0225 0.0225 0.0225 0.0225]';

q=zeros(1,8);
q_dot=zeros(1,8);

% 求初值
Y1 = [100; 100; 100; 100; 100; 100; 100; 100];
Y2 = [1; 2; 3; 4; 5; 6; 7; 10];

[q,q_dot] = SCDPR_Func(tao,tt,Y1,Y2)

% global tao;
% tao=[1 2 3 4 5 6 7 8]';
% global tt;
% tt=[100 100 100 100 100 100 100 100]';
% global Im;
% Im=[0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]';
% global Fv;
% Fv=[1 1 1 1 1 1 1 1]';
% global Fs;
% Fs=[1 1 1 1 1 1 1 1]';
% global R;
% R=[0.0225 0.0225 0.0225 0.0225 0.0225 0.0225 0.0225 0.0225]';
% % 求初值
% Y1 = [100; 100; 100; 100; 100; 100; 100; 100];
% Y2 = [1; 0; 3; 4; 5; 6; 7; 10];
% % 第1个微分方程
% Y11=Y1(1);
% Y21=Y2(1);
% % 解微分方程
% [t,q1] = ode45(@vdp1,[0 0.001],[Y11; Y21]);
% q(1)=q1(end,1);
% % 绘制 $y_1$ 和 $y_2$ 的解对 t 的图。
% figure
% plot(t,q1(:,1),'-o',t,q1(:,2),'-o')
% title('Solution of van der Pol Equation (\mu = 1) with ODE45');
% xlabel('Time t');
% ylabel('Solution y');
% legend('y_1','y_2')
% 
% % 第2个微分方程
% Y12=Y1(2);
% Y22=Y2(2);
% % 解微分方程
% [t,q2] = ode45(@vdp2,[0 0.001],[Y12; Y22]);
% % 绘制 $y_1$ 和 $y_2$ 的解对 t 的图。
% figure
% plot(t,q2(:,1),'-o',t,q2(:,2),'-o')
% title('Solution of van der Pol Equation (\mu = 1) with ODE45');
% xlabel('Time t');
% ylabel('Solution y');
% legend('y_1','y_2')
% 
% % 第3个微分方程
% Y13=Y1(3);
% Y23=Y2(3);
% % 解微分方程
% [t,q3] = ode45(@vdp3,[0 0.001],[Y13; Y23]);
% % 绘制 $y_1$ 和 $y_2$ 的解对 t 的图。
% figure
% plot(t,q3(:,1),'-o',t,q3(:,2),'-o')
% title('Solution of van der Pol Equation (\mu = 1) with ODE45');
% xlabel('Time t');
% ylabel('Solution y');
% legend('y_1','y_2')
% 
% % 第4个微分方程
% Y14=Y1(4);
% Y24=Y2(4);
% % 解微分方程
% [t,q4] = ode45(@vdp4,[0 0.001],[Y14; Y24]);
% % 绘制 $y_1$ 和 $y_2$ 的解对 t 的图。
% figure
% plot(t,q4(:,1),'-o',t,q4(:,2),'-o')
% title('Solution of van der Pol Equation (\mu = 1) with ODE45');
% xlabel('Time t');
% ylabel('Solution y');
% legend('y_1','y_2')
% 
% % 第5个微分方程
% Y15=Y1(5);
% Y25=Y2(5);
% % 解微分方程
% [t,q5] = ode45(@vdp5,[0 0.001],[Y15; Y25]);
% % 绘制 $y_1$ 和 $y_2$ 的解对 t 的图。
% figure
% plot(t,q5(:,1),'-o',t,q5(:,2),'-o')
% title('Solution of van der Pol Equation (\mu = 1) with ODE45');
% xlabel('Time t');
% ylabel('Solution y');
% legend('y_1','y_2')
% 
% % 第6个微分方程
% Y16=Y1(6);
% Y26=Y2(6);
% % 解微分方程
% [t,q6] = ode45(@vdp6,[0 0.001],[Y16; Y26]);
% % 绘制 $y_1$ 和 $y_2$ 的解对 t 的图。
% figure
% plot(t,q6(:,1),'-o',t,q6(:,2),'-o')
% title('Solution of van der Pol Equation (\mu = 1) with ODE45');
% xlabel('Time t');
% ylabel('Solution y');
% legend('y_1','y_2')
% 
% % 第7个微分方程
% Y17=Y1(7);
% Y27=Y2(7);
% % 解微分方程
% [t,q7] = ode45(@vdp7,[0 0.001],[Y17; Y27]);
% % 绘制 $y_1$ 和 $y_2$ 的解对 t 的图。
% figure
% plot(t,q7(:,1),'-o',t,q7(:,2),'-o')
% title('Solution of van der Pol Equation (\mu = 1) with ODE45');
% xlabel('Time t');
% ylabel('Solution y');
% legend('y_1','y_2')
% 
% % 第8个微分方程
% Y18=Y1(8);
% Y28=Y2(8);
% % 解微分方程
% [t,q8] = ode45(@vdp8,[0 0.001],[Y18; Y28]);
% % 绘制 $y_1$ 和 $y_2$ 的解对 t 的图。
% figure
% plot(t,q8(:,1),'-o',t,q8(:,2),'-o')
% title('Solution of van der Pol Equation (\mu = 1) with ODE45');
% xlabel('Time t');
% ylabel('Solution y');
% legend('y_1','y_2')
% 
% function dydt1 = vdp1(t,y,tao,tt,Im,Fv,Fs,R)
% %     global tao;
% %     global tt;
% %     global Im;
% %     global Fv;
% %     global Fs;
% %     global R;
%     
%     tao1=tao(1);
%     tt1=tt(1);
%     Im_inv1=Im(1);
%     Fv1=Fv(1);
%     Fs1=Fs(1);
%     R1=R(1);
% 
%     p = y(1);
%     dp = y(2);
% % 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
%     ddp = -Im_inv1*Fv1*dp-Im_inv1*Fs1*sign(dp)+Im_inv1*tao1-Im_inv1*R1*tt1;
%     dydt1 = [dp; ddp];
% end
% 
% % function dydt1 = vdp1(t,y)
% %     global tao;
% %     global tt;
% %     global Im;
% %     global Fv;
% %     global Fs;
% %     global R;
% %     
% %     tao1=tao(1);
% %     tt1=tt(1);
% %     Im_inv1=Im(1);
% %     Fv1=Fv(1);
% %     Fs1=Fs(1);
% %     R1=R(1);
% % 
% %     p = y(1);
% %     dp = y(2);
% % % 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
% %     ddp = -Im_inv1*Fv1*dp-Im_inv1*Fs1*sign(dp)+Im_inv1*tao1-Im_inv1*R1*tt1;
% %     dydt1 = [dp; ddp];
% % end
% 
% function dydt2 = vdp2(t,y)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
%     
%     tao2=tao(2);
%     tt2=tt(2);
%     Im_inv2=Im(2);
%     Fv2=Fv(2);
%     Fs2=Fs(2);
%     R2=R(2);
% 
%     p = y(1);
%     dp = y(2);
% % 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
%     ddp = -Im_inv2*Fv2*dp-Im_inv2*Fs2*sign(dp)+Im_inv2*tao2-Im_inv2*R2*tt2;
%     dydt2 = [dp; ddp];
% end
% 
% function dydt3 = vdp3(t,y)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
%     
%     tao3=tao(3);
%     tt3=tt(3);
%     Im_inv3=Im(3);
%     Fv3=Fv(3);
%     Fs3=Fs(3);
%     R3=R(3);
% 
%     p = y(1);
%     dp = y(2);
% % 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
%     ddp = -Im_inv3*Fv3*dp-Im_inv3*Fs3*sign(dp)+Im_inv3*tao3-Im_inv3*R3*tt3;
%     dydt3 = [dp; ddp];
% end
% 
% function dydt4 = vdp4(t,y)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
%     
%     tao4=tao(4);
%     tt4=tt(4);
%     Im_inv4=Im(4);
%     Fv4=Fv(4);
%     Fs4=Fs(4);
%     R4=R(4);
% 
%     p = y(1);
%     dp = y(2);
% % 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
%     ddp = -Im_inv4*Fv4*dp-Im_inv4*Fs4*sign(dp)+Im_inv4*tao4-Im_inv4*R4*tt4;
%     dydt4 = [dp; ddp];
% end
% 
% function dydt5 = vdp5(t,y)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
%     
%     tao5=tao(5);
%     tt5=tt(5);
%     Im_inv5=Im(5);
%     Fv5=Fv(5);
%     Fs5=Fs(5);
%     R5=R(5);
% 
%     p = y(1);
%     dp = y(2);
% % 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
%     ddp = -Im_inv5*Fv5*dp-Im_inv5*Fs5*sign(dp)+Im_inv5*tao5-Im_inv5*R5*tt5;
%     dydt5 = [dp; ddp];
% end
% 
% function dydt6 = vdp6(t,y)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
%     
%     tao6=tao(6);
%     tt6=tt(6);
%     Im_inv6=Im(6);
%     Fv6=Fv(6);
%     Fs6=Fs(6);
%     R6=R(6);
% 
%     p = y(1);
%     dp = y(2);
% % 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
%     ddp = -Im_inv6*Fv6*dp-Im_inv6*Fs6*sign(dp)+Im_inv6*tao6-Im_inv6*R6*tt6;
%     dydt6 = [dp; ddp];
% end
% 
% 
% function dydt7 = vdp7(t,y)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
%     
%     tao7=tao(7);
%     tt7=tt(7);
%     Im_inv7=Im(7);
%     Fv7=Fv(7);
%     Fs7=Fs(7);
%     R7=R(7);
% 
%     p = y(1);
%     dp = y(2);
% % 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
%     ddp = -Im_inv7*Fv7*dp-Im_inv7*Fs7*sign(dp)+Im_inv7*tao7-Im_inv7*R7*tt7;
%     dydt7 = [dp; ddp];
% end
% 
% function dydt8 = vdp8(t,y)
%     global tao;
%     global tt;
%     global Im;
%     global Fv;
%     global Fs;
%     global R;
%     
%     tao8=tao(8);
%     tt8=tt(8);
%     Im_inv8=Im(8);
%     Fv8=Fv(8);
%     Fs8=Fs(8);
%     R8=R(8);
% 
%     p = y(1);
%     dp = y(2);
% % 	ddp = -Im_inv*Fv*dp-Im_inv*Fs*sign(dp)+Im_inv*tao-Im_inv*R*tt;
%     ddp = -Im_inv8*Fv8*dp-Im_inv8*Fs8*sign(dp)+Im_inv8*tao8-Im_inv8*R8*tt8;
%     dydt8 = [dp; ddp];
% end