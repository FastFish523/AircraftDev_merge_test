clear
close all
clc
a=load('air.dat');
for i=2:10:60
    index=i/0.005;
    selectedA = a(index,2:20);
    a11=selectedA(1);
    a12=selectedA(2);
    a13=selectedA(3);
    a14=selectedA(4);
    a15=selectedA(5);
    a16=selectedA(6);

    a21=selectedA(7);
    a22=selectedA(8);
    a23=selectedA(9);
    a24=selectedA(10);
    a25=selectedA(11);
    a26=selectedA(12);

    a31=selectedA(13);
    a32=selectedA(14);
    a33=selectedA(15);
    a34=selectedA(16);
    a35=selectedA(17);
    a36=selectedA(18);
    V=selectedA(19);

    flag = -a22*a34-a24;

    if flag>=0
        Km = (-a25*a34+a35*a24)/(a22*a34+a24);
        Tm = 1/(-a24-a22*a34)^0.5;
        Zetam = (-a22+a34)/2/(-a24-a22*a34)^0.5;
        T1=a25/(a25*a34-a35*a24);

        tf_wz = tf([Km*T1 ,Km],[Tm*Tm,2*Tm*Zetam,1]);
        kwz=-0.05;
        figure(1)
        margin(kwz*tf_wz);
        hold on
        [Gm,Pm,Wcg,Wcp] = margin(tf_wz*kwz);
        t= Pm/Wcp/57.3

        tf_wz_bihuan = feedback(tf_wz,kwz,-1);
        tf_ny_open = tf_wz_bihuan*tf([V],[T1,1]);
        tf_fy_open = tf_wz_bihuan*tf([1],[1,0]);

        kp=-0.0001;
        ki=-0.005;
        gc = tf([kp,ki],[1,0]);
        figure(2)
        margin(gc*tf_ny_open);
        hold on
        tf_ny_bihuan = feedback(gc*tf_ny_open,1,-1);
        figure(3)
        step(tf_ny_bihuan)
        hold on
        
%         kp1=-0.1;
%         ki1=-80;
%         gc1 = tf([kp1,ki1],[1,0])
%         figure(4)
%         margin(gc1*tf_fy_open)
%         hold on
%         tf_fy_bihuan = feedback(gc1*tf_fy_open,1,-1)
%         figure(5)
%         step(tf_fy_bihuan)
%         hold on
    end
    if flag<0

    end
end

