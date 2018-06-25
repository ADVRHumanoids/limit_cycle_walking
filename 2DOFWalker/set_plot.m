fig1 = figure(1);
set(fig1,'Position',[2007 1 1107 973]);  %[2400, 400,560,420]


for i = 1:n_link
Link(i) = plot(0,0,'LineWidth',2); grid on; hold on;
end
lims = 3;
xlim([-lims lims]);
ylim([-lims lims]);

xLineTerrain = xlim;
yLineTerrain = tan(alfa) * (xLineTerrain);
LineTerrain = plot(xLineTerrain,yLineTerrain);
handleQuiver = quiver(0,0);


p1plot = plot(0,0,'o');
p2plot = plot(0,0,'o');

%===========================================
for i = 1:n_link
set(Link(i),'xdata',Links(i,1,:),'ydata',Links(i,2,:));
end
%===========================================
fig2 = figure(2);
set(fig2,'Position',[3300, 400,560,420])
for loop = 1:size(q,1)
plot_q(loop) = plot(0,0); hold on;
xlim([0 100]);
ylim([-2 10]);
end


flag = 0;
flag_plot = 0;
time_record = 0;
q_record = zeros(length(q),1);