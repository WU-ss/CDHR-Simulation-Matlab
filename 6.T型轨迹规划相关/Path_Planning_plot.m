xd_x=out.xd(:,1);
xd_y=out.xd(:,2);
xd_z=out.xd(:,3);

xd_dot_x=out.xd_dot(:,1);
xd_dot_y=out.xd_dot(:,2);
xd_dot_z=out.xd_dot(:,3);

xd_ddot_x=out.xd_ddot(:,1);
xd_ddot_y=out.xd_ddot(:,2);
xd_ddot_z=out.xd_ddot(:,3);

t_now=out.t_now;

figure
scatter3(xd_x,xd_y,xd_z,10,t_now,'filled');
xlabel('x轴/m')
ylabel('y轴/m')
zlabel('z轴/m')
grid on
title('T型路径规划得到的轨迹');
h = colorbar;
set(get(h,'label'),'string','时间t/s');%给颜色栏命名
xlim([-7.5 7.5]) %设置坐标轴刻度取值范围
ylim([-5.5 5.5])
zlim([0 6])

figure
subplot(3,1,1)
plot(t_now,xd_x,'k','LineWidth',3)
title('T型路径规划得到的x轴位置');
xlabel('时间t(s)')
ylabel('xd x(m)')
grid on;
subplot(3,1,2)
plot(t_now,xd_y,'k','LineWidth',3)
title('T型路径规划得到的y轴位置');
xlabel('时间t(s)')
ylabel('xd y(m)')
grid on;
subplot(3,1,3)
plot(t_now,xd_z,'k','LineWidth',3)
title('T型路径规划得到的z轴位置');
xlabel('时间t(s)')
ylabel('xd z(m)')
grid on;

figure
subplot(3,1,1)
plot(t_now,xd_dot_x,'b','LineWidth',3)
title('T型路径规划得到的x轴速度');
xlabel('时间t(s)')
ylabel('xd dot x(m/s)')
grid on;
subplot(3,1,2)
plot(t_now,xd_dot_y,'b','LineWidth',3)
title('T型路径规划得到的y轴速度');
xlabel('时间t(s)')
ylabel('xd dot y(m/s)')
grid on;
subplot(3,1,3)
plot(t_now,xd_dot_z,'b','LineWidth',3)
title('T型路径规划得到的z轴速度');
xlabel('时间t(s)')
ylabel('xd dot z(m/s)')
grid on;

figure
subplot(3,1,1)
plot(t_now,xd_ddot_x,'r','LineWidth',3)
title('T型路径规划得到的x轴加速度');
xlabel('时间t(s)')
ylabel('xd ddot x(m/s)')
grid on;
subplot(3,1,2)
plot(t_now,xd_ddot_y,'r','LineWidth',3)
title('T型路径规划得到的y轴加速度');
xlabel('时间t(s)')
ylabel('xd ddot y(m/s)')
grid on;
subplot(3,1,3)
plot(t_now,xd_ddot_z,'r','LineWidth',3)
title('T型路径规划得到的z轴加速度');
xlabel('时间t(s)')
ylabel('xd ddot z(m/s)')
grid on;


% figure
% plot(t_now,xd_x,'b','LineWidth',3)
% title('T型路径规划得到的位置');
% xlabel('时间t(s)')
% ylabel('x(m)')
% grid on;
% figure
% plot(t_now,xd_dot_x,'b','LineWidth',3)
% title('T型路径规划得到的速度');
% xlabel('时间t(s)')
% ylabel('v(m/s)')
% grid on;
% figure
% plot(t_now,xd_ddot_x,'b','LineWidth',3)
% title('T型路径规划得到的加速度');
% xlabel('时间t(s)')
% ylabel('a(m/s2)')
% grid on;