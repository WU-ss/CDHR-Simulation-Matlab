function [f11,n11] = Twolinks_Dynamic_Func(q1,q2,dq1,dq2,ddq1,ddq2,m1,m2,ml,L1,L2,Ixx1,Iyy1,Izz1,Ixx2,Iyy2,Izz2,Ixx3,Iyy3,Izz3,R)
%求解连杆动力学问题(目前还存在问题)
g=9.81;

R_inv=R';

f44 = [0 0 0]';
n44 = [0 0 0]';

IC11 = [Ixx1 0 0;0 Iyy1 0;0 0 Izz1];
IC22 = [Ixx2 0 0;0 Iyy2 0;0 0 Izz2];
IC33 = [Ixx3 0 0;0 Iyy3 0;0 0 Izz3];

zv = [0 0 1]';

c1=cosd(q1);
c2=cosd(q2);
c3=cosd(0);
s1=sind(q1);
s2=sind(q2);
s3=sind(0);

R01 = [c1 -s1 0;s1 c1 0;0 0 1];
R12 = [c2 -s2 0;s2 c2 0;0 0 1];
R23 = [1 0 0;0 1 0;0 0 1];
R34 = [1 0 0;0 1 0;0 0 1];
R43 = R34';R32 = R23';R21 = R12';R10 = R01';
P10 = [0 0 0]';P21 = [L1 0 -0.125]';P32 = [L2 0 -0.125]';P43 = [0 0 0]';
PC11 = [L1/2 0 -0.125]';PC22 = [L2/2 0 -0.125]';PC33=[0 0 0]';
%外推 0->3
w00 = [0 0 0]';dw00 = [0 0 0]';v00 = [0 0 0]';dv00 = R_inv*[0 0 g]';
%这里写g的话，变量g=9.81，如果写-g，那么变量g=-9.81;
w11 = R10*w00+dq1*zv;
dw11 = R10*dw00+cross(R10*w00,dq1*zv)+ddq1*zv;
dv11 = R10*(cross(dw00,P10)+cross(w00,(cross(w00,P10)))+dv00);
dvc11 = cross(dw11,PC11)+cross(w11,cross(w11,PC11))+dv11;
F11 = m1*dvc11;
N11 = IC11*dw11+cross(w11,IC11*w11); 

w22 = R21*w11+dq2*zv;
dw22 = R21*dw11+cross(R21*w11,dq2*zv)+ddq2*zv;
dv22 = R21*(cross(dw11,P21)+cross(w11,(cross(w11,P21)))+dv11);
dvc22 = cross(dw22,PC22)+cross(w22,cross(w22,PC22))+dv22;
F22 = m2*dvc22;
N22 = IC22*dw22+cross(w22,IC22*w22);

dq3=0;
ddq3=0;
w33 = R32*w22+dq3*zv;
dw33 = R32*dw22+cross(R32*w22,dq3*zv)+ddq3*zv;
dv33 = R32*(cross(dw22,P32)+cross(w22,(cross(w22,P32)))+dv22);
dvc33 = cross(dw33,PC33)+cross(w33,cross(w33,PC33))+dv33;
F33 = ml*dvc33;
N33 = IC33*dw33+cross(w33,IC33*w33);

%内推4->1
f33 = R43*f44+F33;
n33 = N33+R43*n44+cross(PC33,F33)+cross(P43,R43*f44);

f22 = R32*f33+F22;
n22 = N22+R32*n33+cross(PC22,F22)+cross(P32,R32*f33);

f11 = R12*f22+F11;
n11 = N11+R12*n22+cross(PC11,F11)+cross(P21,R12*f22);

f11=-f11;
n11=-n11;
% f11=R*f11;
% n11=R*n11;

end

