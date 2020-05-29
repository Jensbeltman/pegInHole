%% Raster search
clear all; close all; clc;
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaulttextInterpreter','latex');

freq=10;
offset=0;
amp=0.5;
duty=50;
t=0.3:0.001:0.8;
sq_wav=offset+amp*square(2*pi*freq.*t,duty);

x0=10;
y0=10;
width=700;
height=700;

figure
set(gcf,'position',[x0,y0,width,height]);
plot(t,sq_wav);
xlim([0.1 1])
ylim([-0.6 0.6]);
ax = gca;
ax.XAxis.FontSize = 15;
ax.YAxis.FontSize = 15;
xlabel('$x_{\Delta}$');
ylabel('$y_{\Delta}$');

%% Stochastic search
clear all; close all; clc;
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaulttextInterpreter','latex');

n = 3000;
R = 1;
ro = rand(n,1);
r = R*sqrt(ro);
fi = 2*pi*rand(n,1);
w_x = r.*cos(fi);
w_y = r.*sin(fi);

x0=10;
y0=10;
width=700;
height=700;

figure
set(gcf,'position',[x0,y0,width,height]);
plot(w_x,w_y,'*');
ax = gca;
ax.XAxis.FontSize = 15;
ax.YAxis.FontSize = 15;
xlabel('$x_{\Delta}$');
ylabel('$y_{\Delta}$');

%% Spiral search
clear all; close all; clc;
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaulttextInterpreter','latex');

r = 1;
a = 0;
b = 0.1;
n = (r - a)./(b);
th = 2*n*pi; 
Th = linspace(0,th,1000); 
x = (a + b.*Th/(2*pi)).*cos(Th);
y = (a + b.*Th/(2*pi)).*sin(Th);

x0=10;
y0=10;
width=700;
height=700;

figure
set(gcf,'position',[x0,y0,width,height]);
plot(x,y);
ax = gca;
ax.XAxis.FontSize = 15;
ax.YAxis.FontSize = 15;
xlabel('$x_{\Delta}$');
ylabel('$y_{\Delta}$');