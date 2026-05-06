function [a,b,c] = Rev_RZYX(RZYX)
%UNTITLED2 计算欧拉角ZYX的逆解问题
%   此处显示详细说明
r11=RZYX(1,1);
r12=RZYX(1,2);
r13=RZYX(1,3);
r21=RZYX(2,1);
r22=RZYX(2,2);
r23=RZYX(2,3);
r31=RZYX(3,1);
r32=RZYX(3,2);
r33=RZYX(3,3);

cb=sqrt(r11^2+r21^2);
if cb~=0
   b=atan2(-r31,sqrt(r11^2+r21^2))/(2*pi)*360
   a=atan2(r21/cb,r11/cb)/(2*pi)*360
   y=atan2(r32/cb,r33/cb)/(2*pi)*360
else
   b=90
   a=0
   y=atan2(r12,r22)
end

end

