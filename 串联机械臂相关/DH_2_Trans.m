function [T1,T2,T3,T] = DH_2_Trans(DHTable)
%DH2TRANS 此处显示有关此函数的摘要
%   此处显示详细说明
apha0=DHTable(1,1);
apha1=DHTable(2,1);
apha2=DHTable(3,1);
a0=DHTable(1,2);
a1=DHTable(2,2);
a2=DHTable(3,2);
d1=DHTable(1,3);
d2=DHTable(2,3);
d3=DHTable(3,3);
theta1=DHTable(1,4);
theta2=DHTable(2,4);
theta3=DHTable(3,4);
T1=[cos(theta1),-sin(theta1),0,a0;
    sin(theta1)*cos(apha0),cos(theta1)*cos(apha0),-sin(apha0),-sin(apha0)*d1;
    sin(theta1)*sin(apha0),cos(theta1)*sin(apha0),cos(apha0),cos(apha0)*d1;
    0,0,0,1;];
T2=[cos(theta2),-sin(theta2),0,a1;
    sin(theta2)*cos(apha1),cos(theta2)*cos(apha1),-sin(apha1),-sin(apha1)*d2;
    sin(theta2)*sin(apha1),cos(theta2)*sin(apha1),cos(apha1),cos(apha1)*d2;
    0,0,0,1;];
T3=[cos(theta3),-sin(theta3),0,a2;
    sin(theta3)*cos(apha2),cos(theta3)*cos(apha2),-sin(apha2),-sin(apha2)*d3;
    sin(theta3)*sin(apha2),cos(theta3)*sin(apha2),cos(apha2),cos(apha2)*d3;
    0,0,0,1;];
T=T1*T2*T3
end

