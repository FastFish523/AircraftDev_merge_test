clear
clc
close all

data = load('result.dat');
data1 = load('../HXD3530/result.dat');
time = data(:,1);
xg = data(:,2);
yg = data(:,3);
zg = data(:,4);
v = data(:,5);
yaw = data(:,6);
pitch = data(:,7);
roll = data(:,8);
theta = data(:,9);
psi = data(:,10);
alpha = data(:,11);
beta = data(:,12);
accx = data(:,13);
accy = data(:,14);
accz = data(:,15);

mass = data(:,16);
P = data(:,17);

acc_cmd_by = data(:,18);
acc_cmd_bz = data(:,19);


xgt = data(:,20);
ygt = data(:,21);
zgt = data(:,22);

dx = data(:,23);
dy = data(:,24);
dz = data(:,25);

vx = data(:,26);
vy = data(:,27);
vz = data(:,28);

wx = data(:,29);
wy = data(:,30);
wz = data(:,31);

lon = data(:,32);
lat = data(:,33);
h = data(:,34);
lon1 = data1(:,32);
lat1 = data1(:,33);
h1 = data1(:,34);

sigma_evl = data(:,35);
sigma_az = data(:,36);
sigma_evl_dot = data(:,37);
sigma_az_dot = data(:,38);


figure(1)
subplot(2,2,1)
plot(xg, yg)
grid on
hold on
plot(xgt, ygt,'r')
hold on
plot(xgt(end), ygt(end),'*')
title('轨迹线 北 - 天');

subplot(2,2,2)
plot(zg, yg)
grid on
hold on
plot(zgt, ygt,'r')
hold on
plot(zgt(end), ygt(end),'*')
title('轨迹线 东 - 天');

subplot(2,2,3)
plot(zg,xg)
hold on
plot(zgt,xgt,'r')
hold on
plot(zgt(end),xgt(end),'*')
hold on
grid on
title('轨迹线 东 北');

subplot(2,2,4)
plot3(xg,zg,yg)
hold on
plot3(xgt,zgt,ygt,'r')
hold on
plot3(xgt(end),zgt(end),ygt(end),'*')
hold on
grid on
title('轨迹线 北 天 东');


figure(2)
subplot(2,1,1)
plot(time,v)
title('时间vs速度');
subplot(2,1,2)
plot(time,h,'r')
grid on
title('时间vs高度');

figure(3)
subplot(2,2,1)
plot(time,alpha)
hold on
plot(time,pitch,'r')
hold on
plot(time,theta,'c')
grid on
legend('攻角','俯仰角','速度倾角')
title('时间vs纵向角度');

subplot(2,2,2)
plot(time,beta,'r')
hold on
plot(time,yaw,'b')
hold on
plot(time,psi,'m')
grid on
legend('侧滑','偏航角','速度偏角')
title('时间vs侧向角度');

subplot(2,2,3)
plot(time,roll)
grid on
title('时间vs滚转角');

subplot(2,2,4)
plot(time,dx,'r')
hold on
plot(time,dy,'k')
hold on
plot(time,dz)
grid on
title('时间vs舵偏');
legend('dx','dy','dz')

figure(4)
subplot(2,1,1)
plot(time,vx,'r')
hold on
plot(time,vy,'k')
hold on
plot(time,vz)
grid on
title('时间vs速度分量');
legend('vx','vy','vz')

subplot(2,1,2)
plot(time,wx,'r')
hold on
plot(time,wy,'k')
hold on
plot(time,wz)
grid on
title('时间vs角速度分量');
legend('wx','wy','wz')

figure(5)
subplot(2,1,1)
plot(time,accx,'r')
grid on
title('时间vs前向弹体实时加速度');
subplot(2,1,2)
plot(time,accy,'g')
hold on
plot(time,accz,'k')
hold on
plot(time,acc_cmd_by,'-.')
hold on
plot(time,acc_cmd_bz,'--')
grid on
legend('上向实时加速度','右向实时加速度','体系法相过载指令','体系侧向过载指令')
title('时间vs上右弹体实时加速度');

figure(6)
subplot(2,1,1)
plot(time,mass)
grid on
title('时间vs质量');
subplot(2,1,2)
plot(time,P)
grid on
title('时间vs推力');


figure(7)
subplot(1,2,1)
plot(time,sigma_evl_dot,'k')
grid on
title('时间vs视线倾角速度');
subplot(1,2,2)
plot(time,sigma_az_dot,'k')
grid on
title('时间vs视线偏角速度');

figure(8)
% 创建二维地图
subplot(2,1,1)
geoplot(lat, lon, 'r-', 'LineWidth', 2)
hold on
geoplot(lat1, lon1, 'b-', 'LineWidth', 2)
hold on
geoscatter(lat([1 end]), lon([1 end]), 50, 'r', 'filled')
hold on
geoscatter(lat1([1 end]), lon1([1 end]), 50, 'b', 'filled')
geobasemap topographic
title('二维轨迹图')
% 高度剖面
subplot(2,1,2)
plot(lon, h, 'r-', 'LineWidth', 2)
hold on
plot(lon1, h1, 'b-', 'LineWidth', 2)
xlabel('经度 (°)')
ylabel('高度 (m)')
title('高度剖面')
grid on


fig = uifigure;
gl = geoglobe(fig);
geoplot3(gl,lat,lon,h,'r');
hold(gl, 'on')
geoplot3(gl,lat1,lon1,h1,'b');

