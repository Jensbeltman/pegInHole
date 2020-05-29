%% example script
clear all
close all
clc

path('apimex',path)
path('Mujoco_lib',path)
dt=1/100;

% connect to Haptix 
mj_connect

% Move robot to home position in Joint space
home = [0 -pi/2 -pi/2 -pi/2 pi/2 0];
% the function generates a lin. path from mesured joint pos to desired (q)
% in time spec. in [s]
JmoveM(home,2);


mj_close

%% 
clear
clc
close all


% Variables
    c = 10;         % Iteration counter / time step

% Constants
    p0 = [0, 0];    % Raster start position: (x_0, y_0, z_0, rx_0, ry_0, rz_0)
    n = 4;         % Number of raster scans
    l = 9;%7.3;          % Raster length
    w = 2.9;          % Raster width
    s = 1;         % Search speed (cartesian)
    f = 2;        % Update frequency

        % Find the length between raster search and the total length 
    if not(mod(n,2)) % Even
        l_step = l/(n-1);
        l_total = (n*abs(w))+l;
    else % Uneven
        l_step = l/n;
        l_total = ((n+1)*abs(w))+l;
    end

        % Workpiece travel distance per itaration
    l_delta = s/f;

        % The percentage of the width of the raster contra width and step
        % length
    q_raster = abs(w)/(abs(w) + l_step);

x = [];
y = [];
for c = 1:l_total/l_delta
% Variables
        % Search iterator to keep track of the workpiece
    if mod(c,l_total/l_delta) > l_total/l_delta
       c_local = l_total/l_delta - mod(c,l_total/l_delta);
    else 
       c_local = mod(c,l_total/l_delta);
    end

        % q_new is the ration of the traveled distance of the width+l_step, and
        % can be used to determine if the workpiece should travel in the x- or
        % y- direction. 
    q_new = c_local*l_delta/(abs(w)+l_step);
    h = floor(q_new);
    d = abs(q_new);

    if d - h > q_raster % True if the x-value should change
        x_delta = (h+ (d-h-q_raster)/(1-q_raster))*l_step;
        %[(d-q_raster) (d-q_raster)/(1-q_raster) 1 x_delta]
    else
        x_delta = h*l_step;
        %[(d-q_raster) (d-q_raster)/(1-q_raster) 0 x_delta]
    end
    
    if d - h > q_raster % False if the y values should change
        if (mod(h,2) == 0 )  % If h is even
            y_delta = w ;
        else
            y_delta = 0;
        end
        %[mod(c, 2*l_total/l_delta) 0 y_delta]
    else % These doesnt work
        if (mod(h,2) == 0 )  % If h is even, increase y
            y_delta = (d-h-q_raster)/(1-q_raster)*w;
        else    % Decrease y 
            y_delta = - (d-h-q_raster)/(1-q_raster)*w;
        end
        %[2*l_total/l_delta mod(c, 2*l_total/l_delta) 1 y_delta ] 
    end
    
    
    delta = [ c_local (d-h > q_raster) x_delta y_delta q_new c*l_delta]
    
%         % Calculate the workpiece offset for each iteration
%     if d-h > q_raster
% %        x_delta = (h + (d-q_raster)/(1-q_raster))*l_step;
%         x_delta = h*l_step;
%         [(d-q_raster) (1-q_raster) (d-q_raster)/(1-q_raster) 1]
%     else
%         y_delta = mod(c,3);
%         x_delta = 0;
%         [(d-q_raster) (1-q_raster) (d-q_raster)/(1-q_raster) 0]
%         
%     end
% 
%     if mod(c,2*l_total/l_delta) > l_total/l_delta
%         y_delta = 2*l_total/l_delta - mod(c, 2*l_total/l_delta);
%         [2*l_total/l_delta mod(c, 2*l_total/l_delta) 1 y_delta ] 
%     else
%         y_delta = mod(c, 2*l_total/l_delta);
%         [mod(c, 2*l_total/l_delta) 0 y_delta]
%     end

% 
    x(end+1) = x_delta;
    y(end+1) = y_delta;
end

%%
% Plot
plotRaster(p0, n, w, l_step)
hold on
%plot(p0(1),p0(2),'*')
plot(x,y,'*')
xlim([-10 9])
ylim([-10 9])
%plot(x)

function plotRaster(p0, n, w, l_step)
X = [p0(1)];
Y = [p0(2)];
for i = 1:n
    if i == n       % If last step
        if mod(n,2) % Uneven
            X(end+1)= X(end);
            Y(end+1)= Y(end)+ w;
            X(end+1)= X(end)+l_step;
            Y(end+1)= Y(end);
            X(end+1)= X(end);
            Y(end+1)= Y(end)- w;
        else        % Even
            X(end+1)= X(end); 
            Y(end+1)= Y(end)-w;
        end
    elseif mod(i,2) % Uneven
        X(end+1)= X(end);
        Y(end+1)= Y(end)+ w;
        X(end+1)= X(end)+l_step;
        Y(end+1)= Y(end);
    else        % Even
        X(end+1)= X(end); 
        Y(end+1)= Y(end)-w;
        X(end+1)= X(end)+l_step;
        Y(end+1)= Y(end);
    end
end
    plot(X, Y, 'b')
 %   xlim([p0(1)-1 l_step*n+1])
 %   ylim([p0(2)-1 w+1])
end