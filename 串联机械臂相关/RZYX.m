function [RAB] = RZYX(a,b,y)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
rz=[cosd(a),-sind(a),0;sind(a),cosd(a),0;0,0,1;];
ry=[cosd(b),0,sind(b);0,1,0;-sind(b),0,cosd(b);];
rx=[1,0,0;0,cosd(y),-sind(y);0,sind(y),cosd(y);];

RAB=rz*ry*rx
end

