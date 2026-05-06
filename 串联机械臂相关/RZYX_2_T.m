function [T] = RZYX_2_T(a,b,c,P)
%RZYZ_2_T 此处显示有关此函数的摘要
%   此处显示详细说明
RAB=RZYX(a,b,c);
T=[RAB P;0 0 0 1]
end

