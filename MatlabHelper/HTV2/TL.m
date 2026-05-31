clear
clc
close all

dt1u=0.5;
T1=3;
dt1d=0.5;
T2=6.9;
dt2d=0.1;

t1 = dt1u;
t2=dt1u+T1;
t3=dt1u+T1+dt1d;
t4=dt1u+T1+dt1d+T2;
t5=dt1u+T1+dt1d+T2+dt2d;
F1= 220 * 1000;
F2= 135 * 1000;
Isp = 245.0
m=726;
for i=1:1:130
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
       FF = -F2/dt2d*(t-t5);
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