clear
clc
close all
i=1;

for Ma=[0.2 1.2 2 4 6 8]
    if Ma < 0.1
        Ma = 0.1;  % 避免除零
    end
    CN =0.3+ 0.6*Ma^2 ./ (1 + 0.8*Ma^4) + 4.0 ./ sqrt(1 + (Ma^2 - 1).^2);

    ma(i)=Ma;
    cn(i)=CN;
    i=i+1;
end
figure(1)
plot(ma,cn)
title('基础系数随马赫数变化')


rho = 0.86;

S=3.2;

beta=0;
cg_cf = 0.5;
dx = 0;
dy = 0;
dz = 0;
rudder_rate = 1;
j=1;
for Ma=[0.2]
    V = Ma*340;
    CN = cn(j);
    CA0 = 0.02 *CN;
    k   = 0.3 *CN;
    i=1;
    for alpha=-30/57.3:0.01:30/57.3
        beta = 0;
        CD = -(CA0 + k * (alpha * alpha + beta * beta))  *0.5*rho*V*V*S;
        CL = CN * alpha*0.5*rho*V*V*S;
        CZ = -CN * beta * 0.5*rho*V*V*S;
        
        l = - CN*dx*cg_cf * 0.05;
        m = -CL*cg_cf - CN*dz*cg_cf*rudder_rate;
        n = CZ*cg_cf - CN*dy*cg_cf*rudder_rate;

        A(i) = alpha*57.3;
        L(i) = CL;
        D(i) = -CD;
        Z(i) = CZ;
        Cl(i) = l;
        Cm(i) = m;
        Cn(i) = n;
        i=i+1;
    end
    
    leg(j) = string(Ma);
    figure(2)
    plot(A,L./D)
    title('升阻比曲线')
    legend(leg)
    hold on
    figure(3)
    plot(A,L)
    title('升力系数曲线')
    legend(leg)
    hold on
    figure(4)
    plot(A,D)
    title('阻力系数曲线')
    legend(leg)
    hold on
    figure(5)
    plot(A,Z)
    title('侧力系数曲线')
    legend(leg)
    hold on
    figure(6)
    plot(A,Cl)
    title('滚转力矩系数曲线')
    legend(leg)
    hold on
    figure(7)
    plot(A,Cm)
    title('俯仰力矩系数曲线')
    legend(leg)
    hold on
    figure(8)
    plot(A,Cn)
    title('偏航力矩系数曲线')
    legend(leg)
    hold on
    
    j=j+1;
end
