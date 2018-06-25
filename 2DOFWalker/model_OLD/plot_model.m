%==================simulation model of a 2 link walker extended========================
syms m1 m2
syms vc1_0 vc2_0
syms w01_0 w12_0 w12_1
syms R1_0 R2_1 R2_0
syms rp1_0 rc1_0
syms rp2_0 rc2_0 
syms rp1_1 rc1_1 
syms rp2_2 rc2_2
syms l1 lc1 l2 lc2
syms q1(t) q2(t) q1_dot(t) q2_dot(t) q1_Ddot(t) q2_Ddot(t)
syms z1(t) z2(t) z1_dot(t) z2_dot(t) z1_Ddot(t) z2_Ddot(t)


Link1 = plot(0,0,'LineWidth',2); grid on; hold on;
Link2 = plot(0,0,'LineWidth',2); grid on; hold on;
xlim([-5 5]);
ylim([-5 5]);

l1 = subs(l1,1);
lc1 = subs(lc1,l1/2);
l2 = subs(l2,1);
lc2 = subs(lc2,l2/2);

Base = [0; 0; 0];



for time = 1:0.5:10


j = pi/2 *time/10;
% j = pi/4;
k =  pi *time/10;
% k = pi/2;

q1(t) = subs(q1(t), q1, j);  
q2(t) = subs(q2(t), q2, -k);  



kinematics;


     

     xLink1 = [Base(1) rp1_0(1)]; %
     yLink1 = [Base(2) rp1_0(2)];

     xLink2 = rp1_0(1) + [Base(1) rp2_0(1)]; %
     yLink2 = rp1_0(2) + [Base(2) rp2_0(2)];
     
     set(Link1,'xdata',xLink1,'ydata',yLink1);
     set(Link2,'xdata',xLink2,'ydata',yLink2);
   
     
     drawnow;

end
