clc;
clear;

a1=[-7.5 -5.5 6]';
a2=[-7.5 -5.5 6]';
a3=[-7.5 5.5 6]';
a4=[-7.5 5.5 6]';
a5=[7.5 5.5 6]';
a6=[7.5 5.5 6]';
a7=[7.5 -5.5 6]';
a8=[7.5 -5.5 6]';
% bi的定义
b1=[0.5 -0.5 3]';
b2=[-0.5 0.5 4]';
b3=[-0.5 -0.5 3]';
b4=[0.5 0.5 4]';
b5=[-0.5 0.5 3]';
b6=[0.5 -0.5 4]';
b7=[0.5 0.5 3]';
b8=[-0.5 -0.5 4]';

figure
plot_cuboid([-7.5,-5.5,0],[7.5,5.5,6]);
hold on
plot_cuboid([-0.5,-0.5,3],[0.5,0.5,4]);
hold on
plot3(a1(1),a1(2),a1(3),'*','color','k','markersize',10, 'linewidth', 2);
hold on
plot3(a2(1),a2(2),a2(3),'*','color','k','markersize',10, 'linewidth', 2);
hold on
plot3(a3(1),a3(2),a3(3),'*','color','k','markersize',10, 'linewidth', 2);
hold on
plot3(a4(1),a4(2),a4(3),'*','color','k','markersize',10, 'linewidth', 2);
hold on
plot3(a5(1),a5(2),a5(3),'*','color','k','markersize',10, 'linewidth', 2);
hold on
plot3(a6(1),a6(2),a6(3),'*','color','k','markersize',10, 'linewidth', 2);
hold on
plot3(a7(1),a7(2),a7(3),'*','color','k','markersize',10, 'linewidth', 2);
hold on
plot3(a8(1),a8(2),a8(3),'*','color','k','markersize',10, 'linewidth', 2);
hold on

plot3(b1(1),b1(2),b1(3),'*','color','b','markersize',10, 'linewidth', 2);
hold on
plot3(b2(1),b2(2),b2(3),'*','color','b','markersize',10, 'linewidth', 2);
hold on
plot3(b3(1),b3(2),b3(3),'*','color','b','markersize',10, 'linewidth', 2);
hold on
plot3(b4(1),b4(2),b4(3),'*','color','b','markersize',10, 'linewidth', 2);
hold on
plot3(b5(1),b5(2),b5(3),'*','color','b','markersize',10, 'linewidth', 2);
hold on
plot3(b6(1),b6(2),b6(3),'*','color','b','markersize',10, 'linewidth', 2);
hold on
plot3(b7(1),b7(2),b7(3),'*','color','b','markersize',10, 'linewidth', 2);
hold on
plot3(b8(1),b8(2),b8(3),'*','color','b','markersize',10, 'linewidth', 2);
hold on

axis([-8 8 -6 6 0 6]) 

h1=plot3([a1(1),b1(1)],[a1(2),b1(2)],[a1(3),b1(3)],'markersize',10, 'linewidth', 2)
h2=plot3([a2(1),b2(1)],[a2(2),b2(2)],[a2(3),b2(3)],'markersize',10, 'linewidth', 2)
h3=plot3([a3(1),b3(1)],[a3(2),b3(2)],[a3(3),b3(3)],'markersize',10, 'linewidth', 2)
h4=plot3([a4(1),b4(1)],[a4(2),b4(2)],[a4(3),b4(3)],'markersize',10, 'linewidth', 2)
h5=plot3([a5(1),b5(1)],[a5(2),b5(2)],[a5(3),b5(3)],'markersize',10, 'linewidth', 2)
h6=plot3([a6(1),b6(1)],[a6(2),b6(2)],[a6(3),b6(3)],'markersize',10, 'linewidth', 2)
h7=plot3([a7(1),b7(1)],[a7(2),b7(2)],[a7(3),b7(3)],'markersize',10, 'linewidth', 2)
h8=plot3([a8(1),b8(1)],[a8(2),b8(2)],[a8(3),b8(3)],'markersize',10, 'linewidth', 2)
% legend(h1,h2,h3,h4,h5,h6,h7,h8)
% legend([h1,h2,h3,h4,h5,h6,h7,h8],['line1','line2','line3','line4','line5','line6','line7','line8'])
