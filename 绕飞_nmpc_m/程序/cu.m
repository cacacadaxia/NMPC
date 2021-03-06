function cu(X1, YMatrix1, YMatrix2, YMatrix3)
%CREATEFIGURE(X1,YMATRIX1,YMATRIX2,YMATRIX3)
%  X1:  vector of x data
%  YMATRIX1:  matrix of y data
%  YMATRIX2:  matrix of y data
%  YMATRIX3:  matrix of y data

%  Auto-generated by MATLAB on 02-May-2018 22:47:41

% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1,...
    'Position',[0.13 0.709264705882353 0.775 0.205243409310627],...
    'FontSize',12);
box(axes1,'on');
hold(axes1,'all');

% Create multiple lines using matrix input to plot
plot1 = plot(X1,YMatrix1,'Parent',axes1,'LineWidth',1.5,'LineStyle','--');
set(plot1(1),'LineWidth',2,'Color',[0 0.749019622802734 0.749019622802734],...
    'LineStyle','-');
set(plot1(2),'Color',[1 0 0]);
grid on;
% Create title
title('x轴控制量','FontSize',16);

% Create axes
axes2 = axes('Parent',figure1,...
    'Position',[0.13 0.409632352941176 0.775 0.205243409310627],...
    'FontSize',12);
box(axes2,'on');
hold(axes2,'all');

% Create multiple lines using matrix input to plot
plot2 = plot(X1,YMatrix2,'Parent',axes2,'LineWidth',1.5,'LineStyle','--');
set(plot2(1),'LineWidth',2,'Color',[0.854901969432831 0.701960802078247 1],...
    'LineStyle','-');
set(plot2(2),'Color',[1 0 0]);
grid on;
% Create title
title('y轴控制量','FontSize',16);

% Create axes
axes3 = axes('Parent',figure1,...
    'Position',[0.13 0.108735777496839 0.775 0.205243409310627],...
    'FontSize',12);
box(axes3,'on');
hold(axes3,'all');

% Create multiple lines using matrix input to plot
plot3 = plot(X1,YMatrix3,'Parent',axes3,'LineWidth',1.5,'LineStyle','--');
set(plot3(1),'LineWidth',2,'Color',[0.701960802078247 0.780392169952393 1],...
    'LineStyle','-');
set(plot3(2),'Color',[1 0 0]);
grid on;
% Create title
title('z轴控制量','FontSize',16);

% Create xlabel
xlabel({'仿真时间t (s)'},'FontSize',20);

% Create ylabel
ylabel({'控制加速度u (m/s^2)'},'FontSize',20);

