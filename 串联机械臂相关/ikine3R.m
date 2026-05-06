function angles = ikine3R(T)
%输入T0H，输出三个转角
T=T*[1 0 0 -2;0 1 0 0;0 0 1 0;0 0 0 1];

l1=4;
l2=3;
x=T(1,4);
y=T(2,4);
cphi=T(1,1);
sphi=T(2,1);

c2=(x^2+y^2-l1^2-l2^2)/(2*l1*l2);
if -1<=c2&&c2<=1
    s2p=sqrt(1-c2^2);
    s2n=-sqrt(1-c2^2);
    theta2_1=atan2(s2p,c2);%theta2_1
    theta2_2=atan2(s2n,c2);%theta2_2

    if x==0 & y==0
        theta1_1=0;
        theta1_2=0;
        theta3_1=atan2(sphi,cphi)-theta1_1-theta2_1;
        theta3_2=atan2(sphi,cphi)-theta1_2-theta2_2;
    else
        %theta2_1
        k1_1=l1+l2*c2;
        k2_1=l2*s2p;
        theta1_1=atan2(y,x)-atan2(k2_1,k1_1);
        theta3_1=atan2(sphi,cphi)-theta1_1-theta2_1;

        %theta2_2
        k1_2=l1+l2*c2;
        k2_2=l2*s2n;
        theta1_2=atan2(y,x)-atan2(k2_2,k1_2);
        theta3_2=atan2(sphi,cphi)-theta1_2-theta2_2;
    end
    angles=[theta1_1,theta2_1,theta3_1;theta1_2,theta2_2,theta3_2];
else
    angles=NaN(2,3);
end