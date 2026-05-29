clear
clc
close all

dt1u=0.1;
T1=2;
dt1d=0.1;

t1 = dt1u;
t2=t1+T1;
t3=t2+dt1d;

F1= 255 * 1000;
F2= 3.5 * 1000;
Isp0 = 1000.0
Isp1 = 7680.0
Isp = Isp0;
m=450;
for i=1:1:8000/0.01
    t=i*0.01;
    T(i)=t;
    FF=0;
    if t<t1
        FF = F1/dt1u*t;
    elseif t<t2
        FF = F1;
    elseif t<t3
       FF = (F2-F1)/dt1d*(t-t2)+F1;
    else
       FF = F2;
       Isp = Isp1;
    if m<=0
        FF = 0;
    end
    end
    F(i)=FF;
    
    dm = FF/Isp/9.8;
    m=m-dm*0.01;
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