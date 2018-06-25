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
syms I1 I2
syms g
syms theta1 theta2 theta3 theta4 theta5

syms m I l lc


Link1 = plot(0,0,'LineWidth',2); grid on; hold on;
Link2 = plot(0,0,'LineWidth',2); grid on; hold on;
xlim([-3 3]);
ylim([-3 3]);



l = subs(l,1);
lc = subs(lc,0.8);
m = subs(m,0.3);
I = subs(I,0.03);
g = subs(g,9.81);

%======================
m1 = m;
m2 = m;
I1 = I;
I2 = I;
l1 = l;
l2 = l;
lc1 = l-lc;
lc2 = lc;
% q1(t) = pi - q1(t);
% q2(t) = 3*pi/2 - q2(t);
%======================

Base = [0; 0; 0];

Origin = [0; 0; 0];


for time = 1:0.5:10


j =  pi *time/10;
% j = pi/2;

k = pi *time/10; 
% k = pi/2;
if k >= 2*pi
k = k - 2*pi;
end
q1(t) = subs(q1(t), q1, j);
q2(t) = subs(q2(t), q2, k);  


% if q2(t) >= 2*pi
%  q2(t) = q2(t) - 2*pi;
% end
 
kinematics_grizzle_symm;



 xLink2 = Base(1) + [Origin(1) rp2_0(1)];
 yLink2 = Base(2) + [Origin(2) rp2_0(2)];

 xLink1 = xLink2(2) + [Origin(1) rp1_0(1)];
 yLink1 = yLink2(2) + [Origin(2) rp1_0(2)];

 set(Link1,'xdata',yLink1,'ydata',xLink1);
 set(Link2,'xdata',yLink2,'ydata',xLink2);
   
     
drawnow;

end