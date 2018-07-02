fig1 = figure(1);
% set(fig1,'Position',[-1854          20        1211         954]);  %[2400, 400,560,420]
set(fig1,'Position',[ 6   552   560   420]);  %[2400, 400,560,420]

for i = 1:n_link
Link(i) = plot(0,0,'LineWidth',2); grid on; hold on;
end
lims = 3;
xlim([-lims lims]);
ylim([-lims lims]);

xLineTerrainLim = xlim;
yLineTerrainLim = tan(slope) * (xLineTerrainLim);
LineTerrain = plot(xLineTerrainLim,yLineTerrainLim);
handleQuiver = quiver(0,0);


p1plot = plot(0,0,'o');
p2plot = plot(0,0,'o');

%===========================================
for i = 1:n_link
set(Link(i),'xdata',Links(i,1,:),'ydata',Links(i,2,:));
end
%===========================================

fig2 = figure(2);
% set(fig2,'Position',[  -609   503   560   420])
set(fig2,'Position',[      8    42   560   420])

% for loop = 1:size(q,1)


plot_q = plot(0,0); hold on;
plot_q_dot = plot(0,0); hold on;
xlim([0 50]);
ylim([-100 100]);
% end

fig3 = figure(3);
energyPlot = plot(0,0);
xlim([0 50]);
% ylim([-5 100]);
E_record = 0;


time_record = 0;
q_record = zeros(length(q),1);
q_dot_record = zeros(length(q_dot),1);