clear all; close all; clc
%==================simulation model of a 2 link walker extended========================
syms m1 m2
syms q1(t) q2(t) q3(t) q4(t)  q5(t)
syms q1_dot(t) q2_dot(t) q3_dot(t) q4_dot(t) q5_dot(t)
syms q1_Ddot(t) q2_Ddot(t) q3_Ddot(t) q4_Ddot(t) q5_Ddot(t)
syms z1(t) z2(t) z1_dot(t) z2_dot(t) z1_Ddot(t) z2_Ddot(t)
syms I1 I2
syms g
syms alfa
% parent_tree = [0,1,2,2,4];
parent_tree = [0,1];
n_link = length(parent_tree);

q = [q1(t), q2(t), q3(t), q4(t), q5(t)];
q_dot = [q1_dot(t), q2_dot(t), q3_dot(t), q4_dot(t)  q5_dot(t)];
q_Ddot = [q1_Ddot(t), q2_Ddot(t), q3_Ddot(t), q4_Ddot(t), q5_Ddot(t)];

z = [z1(t), z2(t)];
z_dot = [z1_dot(t), z2_dot(t)];
z_Ddot = [z1_Ddot(t), z2_Ddot(t)];

qe = [q, z];
qe_dot = [q_dot, z_dot];
qe_Ddot = [q_Ddot, z_Ddot];

d_GeneralizedCoordinates = diff_t(qe, qe, qe_dot);
Dd_GeneralizedCoordinates = diff_t(qe_dot, qe_dot, qe_Ddot);  

rp_rel = sym(zeros(3,length(parent_tree)));
rp_rel(1,:) = sym('l',[length(parent_tree),1]);

rc_rel = sym(zeros(3,length(parent_tree)));
rc_rel(1,:) = sym('lc',[length(parent_tree),1]);

kinematics_n;

for i = 1:n_link
rp_rel(1,i) = subs(rp_rel(1,i),1);
rc_rel(1,i) = subs(rp_rel(1,i),rp_rel(1,i)/2);
end
% lc1 = subs(lc1,l1/2);
% l2 = subs(l2,1);
% lc2 = subs(lc2,l2/2);

% rp_rel = double(rp_rel);
% rc_rel = double(rc_rel);


%===========================
for i = 1:n_link
Link(i) = plot(0,0,'LineWidth',2); grid on; hold on;
end
lims = 5;
xlim([-lims lims]);
ylim([-lims lims]);
%===========================
for time = 1:0.5:10


% j = pi/2 *time/10;
ind = zeros(n_link,1);
ind(1) = 0;
ind(2) = pi/4*time/10;
ind(3) = 0;
ind(4) = pi/4*time/10;
ind(5) = pi/4*time/10;

for i = 1:n_link
q(i) = subs(q(i),q(i),ind(i));
end


kinematics_n;


for j = 1:2
    Links(1,j,:) = 0 + [0 rp_abs(j,:,1)]; %x of link 1 begin/end
end


for i = 2:n_link
    for j = 1:2
Links(i,j,:) = Links(parent_tree(i),j,2) + [0 rp_abs(j,:,i)]; %x of link 1 begin/end
    end
end

for i = 1:n_link
set(Link(i),'xdata',Links(i,1,:),'ydata',Links(i,2,:));
end    

drawnow;

end
