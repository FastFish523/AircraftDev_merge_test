close all
clear
clc
a=load('air.dat');
for i=2:10:60
    index=i/0.005;
    selectedB = a(index,20:end);
    V=selectedB(1);
    b11=selectedB(2);
    b12=selectedB(3);
    b13=selectedB(4);
    b14=selectedB(5);
    b15=selectedB(6);
    b16=selectedB(7);
    b17=selectedB(8);
    b18=selectedB(9);

    b21=selectedB(10);
    b22=selectedB(11);
    b23=selectedB(12);
    b24=selectedB(13);
    b25=selectedB(14);
    b26=selectedB(15);
    b27=selectedB(16);
    b28=selectedB(17);

    b31=selectedB(18);
    b32=selectedB(19);
    b33=selectedB(20);
    b34=selectedB(21);
    b35=selectedB(22);
    b36=selectedB(23);
    b37=selectedB(24);
    b38=selectedB(25);
    if -b24-b22*b34>=0
        km=(-b25*b34+b35*b24)/(b22*b34+b24);
        tm=1/(-b24-b22*b34)^0.5;
        zeta=(-b22+b34)/(2*(-b24-b22*b34)^0.5);
        t1=(b25)/(b25*b34-b35*b24);
       
        num_wy=[km*t1 km];
        den_wy=[tm*tm 2*tm*zeta 1];
        den_ph=[tm*tm 2*tm*zeta 1 0];
        
        tf_wy=tf(num_wy,den_wy);
        kwy=-0.05;
        figure(1)
        margin(kwy*tf_wy);
        hold on
        [Gm,Pm,Wcg,Wcp] = margin(tf_wy*kwy);
        t= Pm/Wcp/57.3
        
        tf_wy_bihuan = feedback(tf_wy,kwy,-1);
        tf_nz_open = tf_wy_bihuan*tf([-V],[t1,1]);

        kp=0.0001;
        ki=0.005;
        gc = tf([kp ki],[1 0]);
        figure(2)
        margin(gc*tf_nz_open);
        hold on
        tf_nz_bihuan = feedback(gc*tf_nz_open,1,-1);
        figure(3)
        step(tf_nz_bihuan);
        hold on

        
    end
    
end