fig1 = figure(1);
set(fig1,'Position',[2007 1 1107 973]);  %[2400, 400,560,420]

Link1 = plot(0,0,'LineWidth',2); grid on; hold on;
Link2 = plot(0,0,'LineWidth',2); grid on; hold on;

xlim([-3 3]);
ylim([-3 3]);

xLineTerrain = xlim;
yLineTerrain = tan(alfa) * (xLineTerrain);
LineTerrain = plot(xLineTerrain,yLineTerrain);
handleQuiver = quiver(0,0);
%===========================================
set(Link1,'xdata',Links(1,1,:),'ydata',Links(1,2,:));
set(Link2,'xdata',Links(2,1,:),'ydata',Links(2,2,:));
%===========================================
fig2 = figure(2);
set(fig2,'Position',[3300, 400,560,420])
for loop = 1:size(q,1)
plot_q(loop) = plot(0,0); hold on;
xlim([0 100]);
ylim([-2 10]);
end

disp('push a button to continue'); pause;
flag = 0;
flag_plot = 0;
time_record = 0;
q_record = zeros(length(q),1);