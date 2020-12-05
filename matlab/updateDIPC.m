function updateDIPC( state_vars, l1, l2, animaHandler, time )

%updateDIPC Draw the animation of double inverted pendulum on a cart system.
%   

drawnow;

time_display = sprintf('Time: %.4f', time);
% positions
cart_position = [state_vars(1) 0];
pendl1_position = cart_position + l1 * [sin(state_vars(2)), cos(state_vars(2))];
pendl2_position = pendl1_position + l2 * [sin(state_vars(3)), cos(state_vars(3))];
w1x = cart_position(1)-animaHandler.wp*animaHandler.W/2;
w1y = animaHandler.yG;
w2x = cart_position(1)+animaHandler.wp*animaHandler.W/2-animaHandler.wr;
w2y = animaHandler.yG;

% update plot
set(animaHandler.cartHandler, 'Position', ...
    [cart_position(1) - animaHandler.W/2, cart_position(2) - animaHandler.H/2, ...
    animaHandler.W, animaHandler.H]);
set(animaHandler.wheel1Handler, 'Position', ...
    [w1x,w1y,animaHandler.wr,animaHandler.wr])
set(animaHandler.wheel2Handler, 'Position', ...
    [w2x,w2y,animaHandler.wr,animaHandler.wr])
set(animaHandler.mass1Handler, 'Position', ...
    [pendl1_position(1)-animaHandler.m1r/2,pendl1_position(2)-animaHandler.m1r/2,...
    animaHandler.m1r,animaHandler.m1r]);
set(animaHandler.mass2Handler, 'Position', ...
    [pendl2_position(1)-animaHandler.m2r/2,pendl2_position(2)-animaHandler.m2r/2,...
    animaHandler.m2r,animaHandler.m2r]);
set(animaHandler.rod1Handler, ...
    'xdata', [cart_position(1) pendl1_position(1)], ...
    'ydata', [cart_position(2) pendl1_position(2)]');
set(animaHandler.rod2Handler, ...
    'xdata', [pendl1_position(1) pendl2_position(1)], ...
    'ydata', [pendl1_position(2) pendl2_position(2)]');
set(animaHandler.timeTextHandler, 'String', time_display);

% % dimensions
% % L = 2;  % pendulum length
% W = 1*sqrt(M/5);  % cart width
% H = .5*sqrt(M/5); % cart height
% wr = .1; % wheel radius
% m1r = .25*sqrt(m1); % mass 1 radius
% m2r = .25*sqrt(m2); % mass 2 radius
% 
% % positions
% cart_position = [state_vars(1) 0];
% pendl1_position = cart_position + l1 * [sin(state_vars(2)), cos(state_vars(2))];
% pendl2_position = pendl1_position + l2 * [sin(state_vars(3)), cos(state_vars(3))];
% yG = -(wr/2+H/2); % ground vertical position
% w1x = cart_position(1)-.9*W/2;
% w1y = yG;
% w2x = cart_position(1)+.9*W/2-wr;
% w2y = yG;
% 
% plot([-10 10],[yG yG],'w','LineWidth',2)
% hold on
% rectangle('Position',[cart_position(1)-W/2,cart_position(2)-H/2,W,H],...
%     'Curvature',.1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1])
% rectangle('Position',[w1x,w1y,wr,wr],...
%     'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1])
% rectangle('Position',[w2x,w2y,wr,wr],...
%     'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1])
% 
% plot([cart_position(1) pendl1_position(1)],[cart_position(2) pendl1_position(2)],'w','LineWidth',2)
% plot([pendl1_position(1) pendl2_position(1)],[pendl1_position(2) pendl2_position(2)],'w','LineWidth',2)
% 
% rectangle('Position',[pendl1_position(1)-m1r/2,pendl1_position(2)-m1r/2,m1r,m1r],...
%     'Curvature',1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1])
% rectangle('Position',[pendl2_position(1)-m2r/2,pendl2_position(2)-m2r/2,m2r,m2r],...
%     'Curvature',1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1])
% 
% % set(gca,'YTick',[])
% % set(gca,'XTick',[])
% canvasW = 8;
% canvasH = 3;
% xlim([-canvasW/2 canvasW/2]);
% ylim([-canvasH/2 canvasH/2]);
% grid on;
% set(gca,'Color','k','XColor','w','YColor','w')
% % set(gcf,'Position',[10 900 canvasW * 100 canvasH * 100])
% set(gcf,'Position',[10 900 canvasW * 100 canvasH * 100])
% set(gcf,'Color','k')
% set(gcf,'InvertHardcopy','off')   
% 
% % box off
% drawnow();
% 
% hold off;


end

