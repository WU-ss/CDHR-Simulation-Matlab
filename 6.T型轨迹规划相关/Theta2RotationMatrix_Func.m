function [Rxyz] = Theta2RotationMatrix_Func(x,y,z)
% theta2RotationMatrix
% 计算给定XYZ欧拉角对应的旋转矩阵
Rx=[1 0 0;0 cosd(x) -sind(x);0 sind(x) cosd(x)];
Ry=[cosd(y) 0 sind(y);0 1 0;-sind(y) 0 cosd(y)];
Rz=[cosd(z) -sind(z) 0;sind(z) cosd(z) 0;0 0 1];
Rxyz=Rx*Ry*Rz;
end