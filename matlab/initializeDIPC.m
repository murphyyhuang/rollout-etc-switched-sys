function [animaHandler] = initializeDIPC(state_vars, M, m1, m2, l1, l2)
%initializeDIPC Initialize the animation of double inverted pendulum on a cart system.
%   

%% Calculate parameters

canvasW = 8;
canvasH = 3;

% dimensions
% L = 2;  % pendulum length
W = 1*sqrt(M/5);  % cart width
H = .5*sqrt(M/5); % cart height
wr = .1; % wheel radius
wp = .9; % wheel relative position
m1r = .25*sqrt(m1); % mass 1 radius
m2r = .25*sqrt(m2); % mass 2 radius

% positions
cart_position = [state_vars(1) 0];
pendl1_position = cart_position + l1 * [sin(state_vars(2)), cos(state_vars(2))];
pendl2_position = pendl1_position + l2 * [sin(state_vars(3)), cos(state_vars(3))];
yG = -(wr/2+H/2); % ground vertical position
w1x = cart_position(1)-wp*W/2;
w1y = yG;
w2x = cart_position(1)+wp*W/2-wr;
w2y = yG;


%% Initialize Canvas
% set(gca,'YTick',[])
% set(gca,'XTick',[])
xlim([-canvasW/2 canvasW/2]);
ylim([-canvasH/2 canvasH/2]);
grid on;
set(gca,'Color','k','XColor','w','YColor','w')
set(gcf,'Position',[10 900 canvasW * 100 canvasH * 100])
set(gcf,'Color','k')
set(gcf,'InvertHardcopy','off')   


%% Initialize Handler
cartHandler = rectangle('Position',[cart_position(1)-W/2,cart_position(2)-H/2,W,H],...
    'Curvature',.1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1]);
wheel1Handler = rectangle('Position',[w1x,w1y,wr,wr],...
    'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1]);
wheel2Handler = rectangle('Position',[w2x,w2y,wr,wr],...
    'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1]);

rod1Handler = line(...
    'xdata', [cart_position(1) pendl1_position(1)],...
    'ydata', [cart_position(2) pendl1_position(2)],...
    'Color', 'w','LineWidth',2);
rod2Handler = line(...
    'xdata', [pendl1_position(1) pendl2_position(1)],...
    'ydata', [pendl1_position(2) pendl2_position(2)],...
    'Color', 'w','LineWidth',2);

mass1Handler = rectangle('Position',[pendl1_position(1)-m1r/2,pendl1_position(2)-m1r/2,m1r,m1r],...
    'Curvature',1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1]);
mass2Handler = rectangle('Position',[pendl2_position(1)-m2r/2,pendl2_position(2)-m2r/2,m2r,m2r],...
    'Curvature',1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1]);

groundHandler = line(...
    'xdata', [-10 10], ...
    'ydata', [yG yG], ...
    'Color', 'w','LineWidth',2);

time_display = sprintf('Time: %.4f s', 0);
timeTextHandler = text(canvasW/2 - 1, -canvasH/2 + 0.5, time_display, 'Color', 'w');


%% construct animation handler

animaHandler.W = W;
animaHandler.H = H;
animaHandler.wr = wr;
animaHandler.wp = wp;
animaHandler.m1r = m1r;
animaHandler.m2r = m2r;
animaHandler.yG = yG;

animaHandler.cartHandler = cartHandler;
animaHandler.wheel1Handler = wheel1Handler;
animaHandler.wheel2Handler = wheel2Handler;
animaHandler.rod1Handler = rod1Handler;
animaHandler.rod2Handler = rod2Handler;
animaHandler.mass1Handler = mass1Handler;
animaHandler.mass2Handler = mass2Handler;
animaHandler.groundHandler = groundHandler;
animaHandler.timeTextHandler = timeTextHandler;

end

