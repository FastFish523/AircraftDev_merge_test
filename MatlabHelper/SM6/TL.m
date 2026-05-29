clear
clc
close all

dt1u=0.5;
T1=6;
dt1d=0.5;
T2=2;
dt2d=0.5;
T3=30;
dt3d=0.1;


t1 = dt1u;
t2=t1+T1;
t3=t2+dt1d;
t4=t3+T2;
t5=t4+dt2d;
t6=t5+T3;
t7=t6+dt3d;

F1= 174 * 1000;
F2= 110 * 1000;
F3= 22 * 1000;

Isp = 245.0
m=726+135;
for i=1:1:600
    t=i*0.1;
    T(i)=t;
    FF=0;
    if t<t1
        FF = F1/dt1u*t;
    elseif t<t2
        FF = F1;
    elseif t<t3
       FF = (F2-F1)/dt1d*(t-t2)+F1;
    elseif t<t4
       FF = F2;
    elseif t<t5
       FF = (F3-F2)/dt2d*(t-t4)+F2;
    elseif t<t6
       FF=F3;
    elseif t<t7
       FF=-F3/dt3d*(t-t7);
    else
       FF=0;
    end
    F(i)=FF;
    
    dm = FF/Isp/9.8;
    m=m-dm*0.1;
    M(i)=m;
    
end
figure(1)
plot(T,F)
grid on
title("推力变化曲线")
xlabel("时间t")
ylabel("推力N")
figure(2)
plot(T,M)
grid on
title("质量变化曲线")
xlabel("时间t")
ylabel("质量kg")