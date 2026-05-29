clear
close all
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

    tf_wx = tf(b17,[1 -b11]);
    k_wx = -0.5;
    [Gm,Pm,Wcg,Wcp]=margin(tf_wx*k_wx);
    t = Pm/Wcp/57.3
    figure(1)
    margin(tf_wx*k_wx);
    hold on
    tf_wx_close = feedback(tf_wx,k_wx,-1);
    tf_roll_open = tf_wx_close*tf([1],[1,0]);
    kp=-3;
    ki=-0.1;
    gc = tf([kp ki],[1 0]);
    figure(2)
    margin(gc*tf_roll_open);
    hold on
    tf_roll_close = feedback(gc*tf_roll_open,1,-1);
    figure(3)
    step(tf_roll_close);
    hold on

end

