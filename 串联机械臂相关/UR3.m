clc;
clear;
L(1)=Link([0,0.1519,0,pi/2]);
L(2)=Link([0,0,-0.24365,0]);
L(3)=Link([0,0,-0.21325,0]);
L(4)=Link([0,0.11235,0,pi/2]);
L(5)=Link([0,0.08535,0,-pi/2]);
L(6)=Link([0,0.0819,0,0]);
Serial_UR3=SerialLink([L(1),L(2),L(3),L(4),L(5),L(6)]);
Serial_UR3.display() %打印UR3的标准DH表

% figure
% Serial_UR3.plot([0,0,0,0,0,0]);

figure
FS=Serial_UR3.fkine([pi/6,-pi/6,pi/3,-pi/6,pi/6,pi/6])
% Serial_UR3.plot([pi/6,-pi/6,pi/3,-pi/6,pi/6,pi/6]);

q_initial_guess = [0,0,0,0,0,0];  % 初始关节角度估计值
IS=Serial_UR3.ikine(FS,'rlimit',500,q_initial_guess)
 Serial_UR3.plot([0.5236   -0.5236    1.0472   -0.5236    0.5236    0.5236])

%得到旋转矩阵
% Rx=trotx(0.192);
% Ry=troty(3.109);
% Rz=trotz(0.036);
% TT=[-63.78/1000 -201.25/1000 137.28/1000 0]'; %输入平移的xyz
% TR=Rx*Ry*Rz
% %得到齐次变换矩阵
% for i=1:4
%     TR(i,4)=TR(i,4)+TT(i);
% end
% q_initial_guess = [9 9 9 9 9 9];  % 初始关节角度估计值
% IS=Serial_UR3.ikine(TR, q_initial_guess)
% % IS./(pi/180)
% FS=Serial_UR3.fkine(IS)
% 
% Rv=[0.001,-3.166,-0.040]; %旋转向量
% TT=[-118.43/1000 -268.05/1000 157.28/1000 1]'; %输入平移的xyz
% R=rotationVectorToMatrix(Rv); %旋转向量转为旋转矩阵
% R(4,1:3)=[0,0,0];
% R(1:4,4)=TT;
% TR=R
% q_initial_guess = [ -76.28 -83.49 -151.01 -36.32 91.85 20.76];  % 初始关节角度估计值
% IS=Serial_UR3.ikine(TR,q_initial_guess)
% % IS./(pi/180)
% FS=Serial_UR3.fkine(IS)